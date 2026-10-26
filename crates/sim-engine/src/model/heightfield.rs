// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Finite regular height grids, independent of solver storage/cooking.
use super::ModelError;
use super::TriangleMesh;
use std::sync::Arc;

/// Finite surface with local +Y height, rows along +Z and columns along +X.
/// Heights are metres, not normalized values. X/Z extents are centred at zero.
/// Every cell uses the diagonal from (row,col) to (row+1,col+1).
///
/// ```
/// use sim_engine::model::HeightField;
/// let grid = HeightField::new(vec![vec![0., 0.], vec![0.1, 0.1]], 2., 3.)?;
/// assert_eq!(grid.to_triangle_mesh().triangles().len(), 2);
/// # Ok::<(), &'static str>(())
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct HeightField {
    heights_m: Arc<[Vec<f64>]>,
    size_x_m: f64,
    size_z_m: f64,
}
impl HeightField {
    /// Require a finite rectangular grid with at least two rows/columns and
    /// finite positive extents. Zero heights and negative elevations are valid.
    ///
    /// # Errors
    /// Rejects ragged, undersized or oversized grids and nonfinite samples or nonpositive extents.
    pub fn new(heights_m: Vec<Vec<f64>>, size_x_m: f64, size_z_m: f64) -> Result<Self, ModelError> {
        let columns = heights_m.first().map_or(0, Vec::len);
        if heights_m.len() < 2
            || columns < 2
            || heights_m
                .len()
                .checked_mul(columns)
                .is_none_or(|n| n > u32::MAX as usize / 2)
            || !heights_m
                .iter()
                .all(|row| row.len() == columns && row.iter().all(|h| h.is_finite()))
            || ![size_x_m, size_z_m]
                .into_iter()
                .all(|v| v.is_finite() && v > 0.)
        {
            return Err(ModelError::InvalidHeightfield);
        }
        Ok(Self {
            heights_m: heights_m.into(),
            size_x_m,
            size_z_m,
        })
    }
    /// Row-major metre heights: rows advance local Z; columns local X.
    pub fn heights_m(&self) -> &[Vec<f64>] {
        &self.heights_m
    }
    /// Full finite local X extent, metres.
    pub fn size_x_m(&self) -> f64 {
        self.size_x_m
    }
    /// Full finite local Z extent, metres.
    pub fn size_z_m(&self) -> f64 {
        self.size_z_m
    }
    /// Indexed diagnostic/fallback surface with identical samples and diagonals.
    /// This conversion is explicit; native cooking does not tessellate the grid.
    pub fn to_triangle_mesh(&self) -> TriangleMesh {
        let rows = self.heights_m.len();
        let cols = self.heights_m[0].len();
        let mut points = Vec::with_capacity(rows * cols);
        for (row, heights) in self.heights_m.iter().enumerate() {
            for (col, height) in heights.iter().enumerate() {
                points.push([
                    (col as f64 / (cols - 1) as f64 - 0.5) * self.size_x_m,
                    *height,
                    (row as f64 / (rows - 1) as f64 - 0.5) * self.size_z_m,
                ]);
            }
        }
        let mut triangles = Vec::with_capacity((rows - 1) * (cols - 1) * 2);
        for row in 0..rows - 1 {
            for col in 0..cols - 1 {
                let a = (row * cols + col) as u32;
                let b = a + 1;
                let c = a + cols as u32 + 1;
                let d = a + cols as u32;
                triangles.extend([[a, c, b], [a, d, c]]);
            }
        }
        TriangleMesh::new(points, triangles).expect("validated finite grid")
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn validates_rectangles_and_preserves_finite_footprint() {
        let grid =
            HeightField::new(vec![vec![0., 0.], vec![0.2, 0.2], vec![0., 0.]], 2., 4.).unwrap();
        let mesh = grid.to_triangle_mesh();
        assert_eq!(mesh.triangles().len(), 4);
        assert_eq!(mesh.vertices_m()[0], [-1., 0., -2.]);
        assert_eq!(mesh.vertices_m()[5], [1., 0., 2.]);
        for h in [
            vec![],
            vec![vec![0., 0.]],
            vec![vec![0., 0.], vec![0.]],
            vec![vec![0., f64::NAN], vec![0., 0.]],
        ] {
            assert!(HeightField::new(h, 2., 4.).is_err());
        }
    }
}
