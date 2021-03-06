use crate::geometry;

use std::fs::File;
use std::io::Write;

pub struct FrameBuffer {
    pub width: usize,
    pub height: usize,
    pub buffer: Vec<Vec<geometry::Vec3f>>,
}

pub fn create_frame_buffer(width: usize, height: usize) -> FrameBuffer {
    // Pre-allocate the whole buffer
    let line_buffer: Vec<geometry::Vec3f> = vec![geometry::Vec3f::zero(); width];
    let buffer = vec![line_buffer; height];

    FrameBuffer {
        width,
        height,
        buffer, // height * lines
    }
}

impl FrameBuffer {
    #[allow(dead_code)]
    pub fn write_ppm(&self, filename: &str, one_to_255: bool) -> std::io::Result<usize> {
        // Open the file stream and dump
        let mut file = File::create(filename)?;
        // return file.write_all(buffer);

        // Standard PPM header
        file.write_all(format!("P6\n{} {}\n255\n", self.width, self.height).as_bytes())?;

        // Write line by line, probably not needed thanks to buffering, but anyway..
        let write_buffer = self.to_vec(one_to_255);
        file.write_all(&write_buffer)?;
        Ok(0)
    }

    pub fn to_vec(&self, one_to_255: bool) -> Vec<u8> {
        let mut write_buffer = vec![0 as u8; self.width * self.height * 3];
        let mut i_ = 0;
        for i in 0..self.height {
            for j in 0..self.width {
                if one_to_255 {
                    write_buffer[i_..i_ + 3].clone_from_slice(&[
                        quantize(self.buffer[i][j].x),
                        quantize(self.buffer[i][j].y),
                        quantize(self.buffer[i][j].z),
                    ]);
                } else {
                    write_buffer[i_..i_ + 3].clone_from_slice(&[
                        self.buffer[i][j].x as u8,
                        self.buffer[i][j].y as u8,
                        self.buffer[i][j].z as u8,
                    ]);
                }
                i_ += 3;
            }
        }

        return write_buffer;
    }

    #[allow(dead_code)]
    pub fn normalize(&mut self) {
        let mut max = geometry::Vec3f::zero();

        for i in 0..self.height {
            for j in 0..self.width {
                max.x = max.x.max(self.buffer[i][j].x);
                max.y = max.y.max(self.buffer[i][j].y);
                max.z = max.z.max(self.buffer[i][j].z);
            }
        }

        println!(
            "Normalizing framebuffer: max values {:.1} {:.1} {:.1}",
            max.x, max.y, max.z
        );
        let max_val = max.x.max(max.y).max(max.z);
        if max_val > 0. {
            for i in 0..self.height {
                for j in 0..self.width {
                    self.buffer[i][j].scale(1. / max_val);
                }
            }
        }
    }
}

fn quantize(f: f32) -> u8 {
    (255. * f.max(0.).min(1.)) as u8
}
