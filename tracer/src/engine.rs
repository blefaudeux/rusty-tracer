use crate::framebuffer;
use crate::geometry::box_test;
use crate::geometry::min;
use crate::geometry::Ray;
use crate::geometry::{new_vec3f, Vec3f};

extern crate rand;
extern crate rayon;
use crate::engine::rayon::prelude::*;

use rand::Rng;
use std::time::Instant;

enum HitType {
  NoHit,
  Letter,
  Wall,
  Sun,
}

impl std::fmt::Display for HitType {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    match self {
      HitType::NoHit => write!(f, "No Hit"),
      HitType::Letter => write!(f, "Letter"),
      HitType::Wall => write!(f, "Wall"),
      HitType::Sun => write!(f, "Sun"),
    }
  }
}

// Sample the world using Signed Distance Fields.
fn sample_world(position: Vec3f) -> (f32, HitType) {
  let mut distance: f32 = 1e9;
  let mut hit_type: HitType;

  let mut f = position; // Flattened position (z=0)
  f.z = 0.;

  // 15*4+1
  let letters: Vec<u8> = concat!(
    // 15 two points lines
    "5O5_", "5W9W", "5_9_", // P (without curve)
    "AOEO", "COC_", "A_E_", // I
    "IOQ_", "I_QO", // X
    "UOY_", "Y_]O", "WW[W", // A
    "aOa_", "aWeW", "a_e_", "cWiO"
  )
  .as_bytes()
  .to_vec(); // R (without curve)

  let n_letters = letters.len() / 4;
  for i in 0..n_letters {
    let begin = new_vec3f(
      letters[4 * i] as f32 - 79.,
      letters[4 * i + 1] as f32 - 79.,
      0.,
    )
    .scaled(0.5);

    let end = new_vec3f(
      letters[4 * i + 2] as f32 - 79.,
      letters[4 * i + 3] as f32 - 79.,
      0.,
    )
    .scaled(0.5)
      - begin;

    let o = f - begin - end.scaled(min(-min((begin - f).dot(end) / end.squared_norm(), 0.), 1.));

    distance = min(distance, o.squared_norm()); // compare squared distance.
  }

  // Get real distance, not square distance, once all the comparisons are done
  distance = distance.sqrt();

  // Two curves (for P and R in PixaR) with hard-coded locations.
  let curves: [Vec3f; 2] = [new_vec3f(11., 6., 0.), new_vec3f(-11., 6., 0.)];

  for curve in curves.iter() {
    let mut o = f - *curve;

    let new_distance = if o.x > 0. {
      (o.norm() - 2.).abs()
    } else {
      o.y += if o.y > 0. { -2. } else { 2. };
      o.norm()
    };

    distance = min(distance, new_distance);
  }

  distance = (distance.powi(8) + position.z.powi(8)).powf(0.125) - 0.5;
  hit_type = HitType::Letter;

  let room_dist = min(
    // min(A,B) = Union with Constructive solid geometry
    //-min carves an empty space
    -min(
      // Lower room
      box_test(
        position,
        new_vec3f(-30., -0.5, -30.),
        new_vec3f(30., 18., 30.),
      ),
      // Upper room
      box_test(
        position,
        new_vec3f(-25., 17., -25.),
        new_vec3f(25., 20., 25.),
      ),
    ),
    box_test(
      // Ceiling "planks" spaced 8 units apart.
      new_vec3f(position.x.abs() % 8., position.y, position.z),
      new_vec3f(1.5, 18.5, -25.),
      new_vec3f(6.5, 20., 25.),
    ),
  );

  if room_dist < distance {
    distance = room_dist;
    hit_type = HitType::Wall;
  }

  // Everything above 19.9 is light source.
  let sun = 19.9 - position.y;
  if sun < distance {
    distance = sun;
    hit_type = HitType::Sun;
  }

  return (distance, hit_type);
}

// Perform signed sphere marching
// Returns hit_type 0, 1, 2, or 3 and update hit position/normal
struct Hit {
  hit_type: HitType,
  pose: Vec3f,
  normal: Vec3f,
}

fn ray_marching(ray: Ray) -> Hit {
  let mut hit_type = HitType::NoHit;
  let mut no_hit_count = 0;
  let mut hit_position: Vec3f = Vec3f::ones();
  let mut hit_normal: Vec3f = Vec3f::ones();

  let mut shortest_distance; // distance from closest object in world.

  // Signed distance marching.
  let mut total_distance = 0.;
  while total_distance < 100. {
    // Keep marching following the SDF hints, until close enough
    hit_position = ray.orig + ray.dir.scaled(total_distance);
    let hit = sample_world(hit_position);
    shortest_distance = hit.0;

    hit_type = hit.1;
    total_distance += shortest_distance;
    no_hit_count += 1;

    if (shortest_distance < 0.01) || no_hit_count > 99 {
      // We're close enough
      // Now get the normal by computing the gradient, use finite difference
      hit_normal = new_vec3f(
        sample_world(hit_position + new_vec3f(0.01, 0., 0.)).0 - shortest_distance,
        sample_world(hit_position + new_vec3f(0., 0.01, 0.)).0 - shortest_distance,
        sample_world(hit_position + new_vec3f(0., 0., 0.01)).0 - shortest_distance,
      )
      .normalized();
      break;
    }
  }

  return Hit {
    hit_type: hit_type,
    pose: hit_position,
    normal: hit_normal,
  };
}

fn trace_sample(mut ray: Ray, rng: &mut rand::ThreadRng) -> Vec3f {
  let mut color = Vec3f::zero();
  let mut attenuation = Vec3f::ones();

  let light_direction = new_vec3f(0.6, 0.6, 1.).normalized(); // Directional light

  let mut bounce_count = 3;

  while bounce_count > 0 {
    let hit = ray_marching(ray);

    match hit.hit_type {
      HitType::NoHit => break,
      HitType::Letter => {
        ray.dir = ray.dir - hit.normal.scaled(2. * hit.normal.dot(ray.dir));
        ray.orig = ray.orig + ray.dir.scaled(0.1);
        attenuation.scale(0.2); // Attenuation via distance traveled.
      }
      HitType::Wall => {
        let incidence = hit.normal.dot(light_direction);
        let p: f32 = 6.283185 * rng.gen::<f32>();
        let c = rng.gen::<f32>();
        let s = (1. - c).sqrt();
        let g = if hit.normal.z < 0. { -1. } else { 1. };
        let u = -1. / (g + hit.normal.z);
        let v = hit.normal.x * hit.normal.y * u;
        ray.dir = new_vec3f(
          v,
          g + hit.normal.y * hit.normal.y * u,
          -hit.normal.y * p.cos() * s,
        ) + new_vec3f(
          1. + g * hit.normal.x * hit.normal.x * u,
          g * v,
          -g * hit.normal.x * p.sin() * s,
        ) + hit.normal.scaled(c.sqrt());

        ray.orig = hit.pose + ray.dir.scaled(0.1);
        attenuation.scale(0.2);

        if incidence > 0. {
          let check_sun = ray_marching(Ray {
            orig: hit.pose + hit.normal.scaled(0.1),
            dir: light_direction,
            hit_number: 0,
          });

          match check_sun.hit_type {
            HitType::Sun => {
              color += attenuation * new_vec3f(500., 400., 100.).scaled(incidence);
            }
            _ => {}
          }
        }
      }
      HitType::Sun => {
        color += attenuation * new_vec3f(50., 80., 100.);
        break; // Sun Color
      }
    }
    bounce_count -= 1;
  }
  return color;
}

pub fn render(width: usize, height: usize, sample_per_pixel: usize) {
  let position = new_vec3f(-22., 5., 25.);

  let goal = (new_vec3f(-3., 4., 0.) - position).normalized();
  let left = new_vec3f(goal.z, 0., -goal.x)
    .normalized()
    .scaled(1. / width as f32);

  // Cross-product to get the up vector
  let up = goal.cross(left);

  println!("Rendering {}x{}", width, height);
  println!("{} threads used", rayon::current_num_threads());

  let mut frame = framebuffer::create_frame_buffer(width as usize, height as usize);
  let start_time = Instant::now();

  // Distribute the computation over spatially coherent patches
  let patch_size = 16;

  if (frame.height % patch_size != 0) || (frame.width % patch_size != 0) {
    println!("Dimensions mismatch")
    // TODO: Move patch_size to GCD in that case
  }

  let n_height = frame.height / patch_size;
  let n_width = frame.width / patch_size;
  let n_patches = n_height * n_width;

  // Render, distribute the patches over threads
  let render_queue: Vec<Vec<Vec3f>> = (0..n_patches)
    .into_par_iter()
    .map(|p| {
      // Pre-allocate the patch
      let mut buffer: Vec<Vec3f> = Vec::with_capacity(patch_size * patch_size);

      let p_line = p % n_width * patch_size;
      let p_col = p / n_width * patch_size;
      let p_line_end = p_line + patch_size;
      let p_col_end = p_col + patch_size;
      let mut rng_thread = rand::thread_rng();

      // Backproject locally, keep spatial coherency
      for i in p_col..p_col_end {
        for j in p_line..p_line_end {
          let mut color = Vec3f::zero();
          let ray = Ray {
            orig: position,
            dir: (goal
              + left.scaled(j as f32 - width as f32 / 2. + rng_thread.gen::<f32>())
              + up.scaled(i as f32 - height as f32 / 2. + rng_thread.gen::<f32>()))
            .normalized(),
            hit_number: 0,
          };

          for _ in 0..sample_per_pixel {
            color += trace_sample(ray, &mut rng_thread);
          }

          // Reinhard tone mapping
          let reinhard = |a: f32| -> f32 {
            let b = a / sample_per_pixel as f32 + 14. / 241.;
            b / (b + 1.) * 255.
          };

          buffer.push(new_vec3f(
            reinhard(color.x),
            reinhard(color.y),
            reinhard(color.z),
          ));
        }
      }
      buffer
    })
    .collect();

  // Reconstruct the picture in the framebuffer
  let mut p_width = 0;
  let mut p_height;

  for (p, render_patch) in render_queue.iter().enumerate() {
    p_height = (p / n_width) * patch_size;
    let p_height_end = p_height + patch_size;
    let p_width_end = p_width + patch_size;

    let mut k = 0;
    for j in p_height..p_height_end {
      for i in p_width..p_width_end {
        frame.buffer[height - j - 1][width - i - 1] = render_patch[k];
        k += 1;
      }
    }
    p_width = p_width_end % frame.width;
  }

  // Compute render time
  let elapsed = start_time.elapsed();
  let render_time_ms = elapsed.as_secs() * 1_000 + u64::from(elapsed.subsec_nanos()) / 1_000_000;
  let n_samples = width * height * sample_per_pixel as usize;
  let samples_per_sec = n_samples as f32 / render_time_ms as f32 * 1000.;

  println!(
    "Done in {0:.2}s, {1:.1} kSamples per second.",
    render_time_ms / 1000,
    samples_per_sec / 1000.
  );
  match frame.write_ppm("rendering.ppm", false) {
    Ok(_) => {}
    Err(_) => {
      println!("Failed saving the picture");
    }
  }
}
