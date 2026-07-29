mod segment;
mod collide;
mod split;
mod save;
mod triangle;
mod idx_triangle;
mod aabb;

pub mod primitives;
pub mod mesh;
pub mod utils;

#[cfg(feature = "obj")]
pub mod export;

#[cfg(feature = "obj")]
extern crate obj;

use std::fmt::Display;

use log::debug;
use nalgebra::{Point3, Vector3, Unit};

// Crate uses
pub use collide::Collide;
pub use save::Save;
pub use split::Split;
pub use segment::{Segment, SplitEdges};
pub use triangle::Triangle;
pub use idx_triangle::IdxTriangle;

use mesh::{Faces, Mesh, Norms, Verts};

use crate::primitives::{Normal, PrimitiveIdx, Vertex};

pub type Dir3 = Unit<Vector3<f64>>;

pub type VTNIndex = (usize, Option<usize>, Option<usize>);

pub struct Inventory {
    pub meshes: Vec<Mesh>,
    pub verts: Verts,
    pub norms: Norms,
    pub faces: Faces,
}

impl Inventory {
    pub fn new() -> Self {
        Self {
            meshes: Vec::new(),
            verts: Verts::new(),
            norms: Norms::new(),
            faces: Faces::new(),
        }
    }

    // Add vertices from a Point3 list
    pub fn add_verts(&mut self, verts: Vec<Point3<f64>>) -> Vec<PrimitiveIdx> {
        let indices = self.verts.allocate_iter(verts);
        debug!("Allocated verts idx: {:?}", indices);
        indices
    }

    // Add vertices from a flat list of f64 values (x, y, z).
    pub fn add_verts_flat(&mut self, verts: Vec<f64>) -> Vec<PrimitiveIdx> {
        let verts_points = verts
            .chunks_exact(3)
            .map(|chunk| Point3::new(chunk[0], chunk[1], chunk[2]))
            .collect::<Vec<_>>();
        self.add_verts(verts_points)
    }

    pub fn add_norms(&mut self, norms: Vec<Dir3>) -> Vec<PrimitiveIdx> {
        let indices = self.norms.allocate_iter(norms);
        debug!("Allocated norms idx: {:?}", indices);
        indices
    }

    // Add normals from a flat list of f64 values (x, y, z).
    pub fn add_norms_flat(&mut self, norms: Vec<f64>) -> Vec<PrimitiveIdx> {
        let norms_vectors = norms
            .chunks_exact(3)
            .map(|chunk| Dir3::new_normalize(Vector3::new(chunk[0], chunk[1], chunk[2])))
            .collect::<Vec<_>>();
        self.add_norms(norms_vectors)
    }

    pub fn add_mesh(&mut self, verts_indices: Vec<[usize; 3]>, norms_indices: Vec<Option<[usize; 3]>>, name: String, mat_name: Option<String>) {
        // Tag IdxTriangle with the correct idx that would be allocated to faces
        let mut idx_tris = Vec::new();
        for ([v1,v2,v3], tri_norms_idx) in verts_indices.iter().zip(norms_indices.iter()) {
            let tri_verts: [Vertex; 3] = [
                Vertex::new(*v1, &self.verts.borrow()),
                Vertex::new(*v2, &self.verts.borrow()),
                Vertex::new(*v3, &self.verts.borrow()),
            ];
            let tri_norms: Option<[Normal; 3]> = tri_norms_idx.map(|[n1,n2,n3]|
            [
                Normal::new(n1, &self.norms.borrow()),
                Normal::new(n2, &self.norms.borrow()),
                Normal::new(n3, &self.norms.borrow()),
            ]);
            let idx_tri = IdxTriangle::new(tri_verts, tri_norms, PrimitiveIdx::Local(0));
            idx_tris.push(idx_tri);
        }
        let polygons = self.faces.allocate_iter(idx_tris);
        debug!("Allocated faces idx for mesh {}: {:?}", name, polygons);

        let mesh = Mesh::from_polygons(
            name.clone(),
            PrimitiveIdx::Global(self.meshes.len()),
            polygons,
            &self.verts,
            &self.norms,
            &self.faces
        );

        self.meshes.push(mesh.with_material(mat_name));
    }

    pub fn add_mesh_standalone<S: AsRef<str> + Display>(&mut self, verts: Vec<Point3<f64>>, norms: Vec<Vector3<f64>>, verts_indices: Vec<[usize; 3]>, norms_indices: Vec<Option<[usize; 3]>>, name: S, mat_name: Option<S>) {
        let verts_idx = self.add_verts(verts);
        let norms_dirs = norms.into_iter().map(|n| Dir3::new_normalize(n)).collect();
        let norms_idx = self.add_norms(norms_dirs);

        let mut idx_tris = Vec::new();
        for ([v1,v2,v3], tri_norms_idx) in verts_indices.iter().zip(norms_indices.iter()) {
            let tri_verts: [Vertex; 3] = [
                Vertex::new(*verts_idx[*v1], &self.verts.borrow()),
                Vertex::new(*verts_idx[*v2], &self.verts.borrow()),
                Vertex::new(*verts_idx[*v3], &self.verts.borrow()),
            ];
            let tri_norms: Option<[Normal; 3]> = tri_norms_idx.map(|[n1,n2,n3]|
            [
                Normal::new(*norms_idx[n1], &self.norms.borrow()),
                Normal::new(*norms_idx[n2], &self.norms.borrow()),
                Normal::new(*norms_idx[n3], &self.norms.borrow()),
            ]);
            let idx_tri = IdxTriangle::new(tri_verts, tri_norms, PrimitiveIdx::Local(0));
            idx_tris.push(idx_tri);
        }
        let polygons = self.faces.allocate_iter(idx_tris);
        debug!("Allocated faces idx for mesh {}: {:?}", name, polygons);

        let mesh = Mesh::from_polygons(
            name.as_ref().to_string(),
            PrimitiveIdx::Global(self.meshes.len()),
            polygons,
            &self.verts,
            &self.norms,
            &self.faces
        );

        self.meshes.push(mesh.with_material(mat_name.map(|s| s.as_ref().to_string())));
    }
}

