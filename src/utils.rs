use std::{io::Read, path::Path};
use anyhow::{anyhow, Result};
use nalgebra::{Point3, Vector3};

use obj::ObjData;
#[cfg(feature = "obj")]
use obj::{Obj, ObjMaterial};

use crate::{Dir3, Inventory, mesh::prune_verts};

#[cfg(feature = "tobj")]
pub fn parse_tobj(obj_path: &Path) -> Inventory {
    todo!()
}

pub fn parse_obj_buf<R: Read>(input: R) -> Result<Inventory> {
    let obj_data = ObjData::load_buf(input).map_err(|e| anyhow!("Loading wavefront data failed: {}", e))?;
    parse_obj(obj_data)
}

#[cfg(feature = "obj")]
pub fn parse_obj_file(obj_path: &Path) -> Result<Inventory> {
    let obj = Obj::load(obj_path).unwrap_or_else(|e| panic!("Loading wavefront file {:?} failed: {}", obj_path, e));
    let obj_data = obj.data;

    parse_obj(obj_data)
}

pub fn parse_obj(obj_data: ObjData) -> Result<Inventory> {
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
