#[cfg(feature = "tobj")]
use std::io::BufRead;
use std::{io::Read, path::Path};
use anyhow::{anyhow, Result, Context};
use nalgebra::{Point3, Vector3};

use crate::{Dir3, Inventory, mesh::prune_verts};

#[cfg(feature = "obj")]
use obj::ObjData;


#[cfg(feature = "tobj")]
pub fn parse_tobj_file(obj_path: &Path) -> Result<Inventory> {
    let (models, _materials) = tobj::load_obj(obj_path, &tobj::OFFLINE_RENDERING_LOAD_OPTIONS)
        .context(format!("Loading wavefront file {:?} failed", obj_path))?;
    parse_tobj(&models)
}

#[cfg(feature = "tobj")]
pub fn parse_tobj_buf<R: BufRead>(input: &mut R) -> Result<Inventory> {
    use tobj::{MTLLoadResult, LoadError};

    let (models, _materials) = tobj::load_obj_buf(input, &tobj::OFFLINE_RENDERING_LOAD_OPTIONS, |_ml| MTLLoadResult::Err(LoadError::MaterialParseError))
        .context(format!("Loading wavefront data failed"))?;
    parse_tobj(&models)
}

// FIXME:Extracted vertices seem to be incorrect
#[cfg(feature = "tobj")]
pub fn parse_tobj(models: &Vec<tobj::Model>) -> Result<Inventory> {
    let mut inventory = Inventory::new();

    for m in models.iter() {
        let mesh = &m.mesh;
        let verts: Vec<Point3<f64>> = mesh.positions.chunks(3)
            .map(|chunk| Point3::new(chunk[0] as f64, chunk[1] as f64, chunk[2] as f64))
            .collect();
        inventory.add_verts(verts);

        let norms: Vec<Dir3> = mesh.normals.chunks(3)
            .map(|chunk| Dir3::new_normalize(Vector3::new(chunk[0] as f64, chunk[1] as f64, chunk[2] as f64)))
            .collect();
        inventory.add_norms(norms);
    }

    let mut base_vert_pos = 0;
    let mut base_norm_pos = 0;
    for m in models.iter() {
        let mesh = &m.mesh;
        // Check that all faces are triangles
        for fa in mesh.face_arities.iter() {
            if *fa != 3 {
                return Err(anyhow!("Non-triangular face found in mesh: {}", m.name));
            }
        }
        // Extract the vertex and normal indices for each triangle, shifting indices to align with collected in inventory
        let tris_verts: Vec<[usize; 3]> = mesh.indices.chunks(3)
            .map(|chunk| [chunk[0] as usize + base_vert_pos, chunk[1] as usize + base_vert_pos, chunk[2] as usize + base_vert_pos])
            .collect();
        let tris_norms: Vec<Option<[usize; 3]>> = if !mesh.normal_indices.is_empty() {
            mesh.normal_indices.chunks(3)
            .map(|chunk| Some([chunk[0] as usize + base_norm_pos, chunk[1] as usize + base_norm_pos, chunk[2] as usize + base_norm_pos]))
            .collect()
        } else {
            vec![None; tris_verts.len()]
        };

        base_vert_pos += mesh.positions.len() / 3;
        base_norm_pos += mesh.normals.len() / 3;

        inventory.add_mesh(tris_verts, tris_norms, m.name.clone(), Some(m.name.clone()));
    }

    // Vertex deduplication, and remap the vertex idx in faces accordingly
    let _verts_remap = prune_verts(&inventory.verts, &inventory.faces);

    Ok(inventory)
}


#[cfg(feature = "obj")]
pub fn parse_obj_buf<R: Read>(input: R) -> Result<Inventory> {
    let obj_data = ObjData::load_buf(input).map_err(|e| anyhow!("Loading wavefront data failed: {}", e))?;
    parse_obj(obj_data)
}

#[cfg(feature = "obj")]
pub fn parse_obj_file(obj_path: &Path) -> Result<Inventory> {
    use obj::Obj;
    let obj = Obj::load(obj_path).unwrap_or_else(|e| panic!("Loading wavefront file {:?} failed: {}", obj_path, e));
    let obj_data = obj.data;

    parse_obj(obj_data)
}

#[cfg(feature = "obj")]
pub fn parse_obj(obj_data: ObjData) -> Result<Inventory> {
    use obj::ObjMaterial;
    let mut inventory = Inventory::new();
    let verts = obj_data.position
        .iter()
        .map(|vs| {
            let vs_f64: [f64; 3] = vs.map(|v| v as f64);
            Point3::new(vs_f64[0], vs_f64[1], vs_f64[2])
        })
        .collect();
    inventory.add_verts(verts);

    let norms = obj_data.normal
        .iter()
        .map(|vs| {
            let vs_f64: [f64; 3] = vs.map(|v| v as f64);
            Dir3::new_normalize(Vector3::new(vs_f64[0], vs_f64[1], vs_f64[2]))
        })
        .collect();
    inventory.add_norms(norms);

    for obj in obj_data.objects.iter() {
        // NOTE: Collapse groups from an object
        let verts = obj.groups
            .iter()
            .flat_map(|group| {
                group.polys.iter().map(|poly| {
                    let tri_verts: [usize; 3] = poly
                        .0
                        .iter()
                        .map(|idx_tuple| idx_tuple.0)
                        .collect::<Vec<_>>()
                        .try_into()
                        .unwrap();
                    tri_verts
                })
            })
            .collect();

        let norms = obj.groups
            .iter()
            .flat_map(|group| {
                group.polys.iter().map(|poly| {
                    let tri_norms: Option<[usize; 3]> = poly
                        .0
                        .iter()
                        .map(|idx_tuple| idx_tuple.2)
                        .collect::<Option<Vec<_>>>()
                        .and_then(|v| v.try_into().ok());
                    tri_norms
                })
            })
            .collect();

        let mut mat_name = None;
        for group in &obj.groups {
            match &group.material {
                Some(ObjMaterial::Ref(name)) => {
                    if let Some(ref existing) = mat_name {
                        if *existing != *name {

                            return Err(anyhow!(
                                "Multiple material names found for object {}: {} and {}",
                                obj.name, existing, name
                            ));
                        }
                    } else {
                        mat_name = Some(name.clone());
                    }
                },
                None => {},
                _ => return Err(anyhow!("Material description from wavefront obj file not supported")),
            };
        }

        inventory.add_mesh(verts, norms, obj.name.clone(), mat_name);
    }

    // Vertex deduplication, and remap the vertex idx in faces accordingly
    let _verts_remap = prune_verts(&inventory.verts, &inventory.faces);
    Ok(inventory)
}
