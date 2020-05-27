use std::cmp;
use std::ops::{Add, AddAssign, Mul, Neg, Sub};

// TODO: Move to std::simd https://doc.rust-lang.org/1.2.0/std/simd/
#[derive(Debug, PartialEq, PartialOrd, Copy, Clone)]
pub struct Vec3f {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    _pad: f32, // Tentatively enable packed SIMD
}

pub fn new_vec3f(x: f32, y: f32, z: f32) -> Vec3f {
    Vec3f {
        x: x,
        y: y,
        z: z,
        _pad: 0.,
    }
}

#[derive(Debug, PartialEq, PartialOrd, Copy, Clone)]
pub struct Ray {
    pub orig: Vec3f,
    pub dir: Vec3f,
    pub hit_number: u8,
}

pub fn min<T>(v1: T, v2: T) -> T
where
    T: cmp::PartialOrd,
{
    if v1 < v2 {
        return v1;
    } else {
        return v2;
    };
}

impl Vec3f {
    pub fn normalized(&self) -> Vec3f {
        let mut other = *self;
        normalize(&mut other);
        other
    }

    pub fn scale(&mut self, s: f32) {
        self.x *= s;
        self.y *= s;
        self.z *= s;
    }

    pub fn scaled(&self, s: f32) -> Vec3f {
        let mut other = *self;
        other.scale(s);
        other
    }

    pub fn dot(self, other: Vec3f) -> f32 {
        dot(self, other)
    }

    pub fn cross(&self, other: Vec3f) -> Vec3f {
        new_vec3f(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }

    pub fn squared_norm(self) -> f32 {
        dot(self, self)
    }

    pub fn norm(&self) -> f32 {
        self.squared_norm().sqrt()
    }

    // Common values
    pub fn zero() -> Vec3f {
        new_vec3f(0., 0., 0.)
    }

    pub fn ones() -> Vec3f {
        new_vec3f(1., 1., 1.)
    }
}

// Rectangle CSG equation. Returns minimum signed distance from
// space carved by lower_left vertex and opposite rectangle vertex upper_right.
pub fn box_test(position: Vec3f, lower_left: Vec3f, upper_right: Vec3f) -> f32 {
    let lower_left = position - lower_left;
    let upper_right = upper_right - position;

    return -min(
        min(
            min(lower_left.x, upper_right.x),
            min(lower_left.y, upper_right.y),
        ),
        min(lower_left.z, upper_right.z),
    );
}

// Some of the implementations, private
fn normalize(vec: &mut Vec3f) {
    let norm = dot(*vec, *vec).sqrt();
    if norm > 0. {
        vec.scale(1. / norm);
    }
}

impl Add for Vec3f {
    type Output = Vec3f;

    fn add(self, other: Vec3f) -> Vec3f {
        new_vec3f(self.x + other.x, self.y + other.y, self.z + other.z)
    }
}

impl Neg for Vec3f {
    type Output = Vec3f;

    fn neg(self) -> Vec3f {
        new_vec3f(-self.x, -self.y, -self.z)
    }
}

impl AddAssign for Vec3f {
    fn add_assign(&mut self, other: Vec3f) {
        self.x += other.x;
        self.y += other.y;
        self.z += other.z;
    }
}

impl Mul for Vec3f {
    type Output = Self;
    fn mul(self, other: Vec3f) -> Vec3f {
        new_vec3f(self.x * other.x, self.y * other.y, self.z * other.z)
    }
}

impl Sub for Vec3f {
    type Output = Self;
    fn sub(self, other: Vec3f) -> Vec3f {
        new_vec3f(self.x - other.x, self.y - other.y, self.z - other.z)
    }
}

impl std::fmt::Display for Vec3f {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> Result<(), std::fmt::Error> {
        write!(f, "{:.2} {:.2} {:.2}", self.x, self.y, self.z)
    }
}

fn dot(v1: Vec3f, v2: Vec3f) -> f32 {
    (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z) as f32
}
