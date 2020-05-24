use crate::framebuffer;
use crate::geometry::box_test;
use crate::geometry::min;
use crate::geometry::Ray;
use crate::geometry::Vec3f;
use std::time::Instant;

extern crate rand;
use rand::Rng;

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
fn sample_world(position: Vec3f) -> (f64, HitType) {
  let mut distance: f64 = 1e9;
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
    let begin = Vec3f {
      x: letters[4 * i] as f64 - 79.,
      y: letters[4 * i + 1] as f64 - 79.,
      z: 0.,
    }
    .scaled(0.5);

    let end = Vec3f {
      x: letters[4 * i + 2] as f64 - 79.,
      y: letters[4 * i + 3] as f64 - 79.,
      z: 0.,
    }
    .scaled(0.5)
      - begin;

    let o = f - begin - end.scaled(min(-min((begin - f).dot(end) / end.squared_norm(), 0.), 1.));

    distance = min(distance, o.squared_norm()); // compare squared distance.
  }

  // Get real distance, not square distance, once all the comparisons are done
  distance = distance.sqrt();

  // Two curves (for P and R in PixaR) with hard-coded locations.
  let curves: [Vec3f; 2] = [
    Vec3f {
      x: 11.,
      y: 6.,
      z: 0.,
    },
    Vec3f {
      x: -11.,
      y: 6.,
      z: 0.,
    },
  ];

  for curve in curves.iter() {
    let mut o = f - *curve;
    let o_norm = o.norm();

    let new_distance = if o.x > 0. {
      (o_norm - 2.).abs()
    } else {
      o.y += o.y;
      o_norm
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
        Vec3f {
          x: -30.,
          y: -0.5,
          z: -30.,
        },
        Vec3f {
          x: 30.,
          y: 18.,
          z: 30.,
        },
      ),
      // Upper room
      box_test(
        position,
        Vec3f {
          x: -25.,
          y: 17.,
          z: -25.,
        },
        Vec3f {
          x: 25.,
          y: 20.,
          z: 25.,
        },
      ),
    ),
    box_test(
      // Ceiling "planks" spaced 8 units apart.
      Vec3f {
        x: position.x.abs() % 8.,
        y: position.y,
        z: position.z,
      },
      Vec3f {
        x: 1.5,
        y: 18.5,
        z: -25.,
      },
      Vec3f {
        x: 6.5,
        y: 20.,
        z: 25.,
      },
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
fn ray_marching(ray: Ray) -> (HitType, Vec3f, Vec3f) {
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
      hit_normal = Vec3f {
        x: sample_world(
          hit_position
            + Vec3f {
              x: 0.01,
              y: 0.,
              z: 0.,
            },
        )
        .0 - shortest_distance,
        y: sample_world(
          hit_position
            + Vec3f {
              x: 0.,
              y: 0.01,
              z: 0.,
            },
        )
        .0 - shortest_distance,
        z: sample_world(
          hit_position
            + Vec3f {
              x: 0.,
              y: 0.,
              z: 0.01,
            },
        )
        .0 - shortest_distance,
      }
      .normalized();
      break;
    }
  }

  return (hit_type, hit_position, hit_normal);
}

fn trace_sample(mut ray: Ray) -> Vec3f {
  let mut color = Vec3f::zero();
  let mut attenuation = Vec3f::ones();

  let light_direction = Vec3f {
    x: 0.6,
    y: 0.6,
    z: 1.,
  }
  .normalized(); // Directional light

  let mut bounce_count = 3;
  let mut rng = rand::thread_rng();

  while bounce_count > 0 {
    let hit = ray_marching(ray);
    let hit_type = hit.0;
    let hit_pos = &hit.1;
    let hit_normal = &hit.2;
    match hit_type {
      HitType::NoHit => break,
      HitType::Letter => {
        ray.dir = ray.dir - hit_normal.scaled(2. * hit_normal.dot(ray.dir));
        ray.orig = ray.orig + ray.dir.scaled(0.1);
        attenuation.scale(0.2); // Attenuation via distance traveled.
      }
      HitType::Wall => {
        let incidence = hit_normal.dot(light_direction);
        let p: f64 = 6.283185 * rng.gen::<f64>();
        let c = rng.gen::<f64>();
        let s = (1. - c).sqrt();
        let g = if hit_normal.z < 0. { -1. } else { 1. };
        let u = -1. / (g + hit_normal.z);
        let v = hit_normal.x * hit_normal.y * u;
        ray.dir = Vec3f {
          x: v,
          y: g + hit_normal.y * hit_normal.y * u,
          z: -hit_normal.y * p.cos() * s,
        } + Vec3f {
          x: 1. + g * hit_normal.x * hit_normal.x * u,
          y: g * v,
          z: -g * hit_normal.x * p.sin() * s,
        } + hit_normal.scaled(c.sqrt());

        ray.orig = hit.1 + ray.dir.scaled(0.1);
        attenuation.scale(0.2);

        if incidence > 0. {
          let check_sun = ray_marching(Ray {
            orig: *hit_pos + hit_normal.scaled(0.1),
            dir: light_direction,
            hit_number: 0,
          });

          match check_sun.0 {
            HitType::Sun => {
              color += attenuation
                * Vec3f {
                  x: 500.,
                  y: 400.,
                  z: 100.,
                }
                .scaled(incidence);
            }
            _ => {}
          }
        }
      }
      HitType::Sun => {
        color += attenuation
          * Vec3f {
            x: 50.,
            y: 80.,
            z: 100.,
          };
        break; // Sun Color
      }
    }
    bounce_count -= 1;
  }
  return color;
}

pub fn render(width: i64, height: i64, sample_per_pixel: u8) {
  let position = Vec3f {
    x: -22.,
    y: 5.,
    z: 25.,
  };

  let goal = (Vec3f {
    x: -3.,
    y: 4.,
    z: 0.,
  } - position)
    .normalized();
  let left = Vec3f {
    x: goal.z,
    y: 0.,
    z: -goal.x,
  }
  .normalized()
  .scaled(1. / width as f64);

  // Cross-product to get the up vector
  let up = goal.cross(left);
  let mut rng = rand::thread_rng();

  println!("Rendering {}x{}", width, height);
  let mut fb = framebuffer::create_frame_buffer(width as usize, height as usize);
  let start_time = Instant::now();

  for y in 0..height {
    for x in 0..width {
      let mut color = Vec3f::zero();

      for _ in 0..sample_per_pixel {
        color += trace_sample(Ray {
          orig: position,
          dir: (goal
            + left.scaled(x as f64 - width as f64 / 2. + rng.gen::<f64>())
            + up.scaled(y as f64 - height as f64 / 2. + rng.gen::<f64>()))
          .normalized(),
          hit_number: 0,
        });
      }

      // Reinhard tone mapping
      color.scale(1. / sample_per_pixel as f64);
      color.offset(14. / 241.);

      let mut o = color;
      o.offset(1.);

      color = Vec3f {
        x: color.x / o.x,
        y: color.y / o.y,
        z: color.z / o.z,
      }
      .scaled(255.);

      fb.buffer[(height - y - 1) as usize][(width - x - 1) as usize] = color;
    }
  }

  // Compute render time
  let elapsed = start_time.elapsed();
  let render_time_ms = elapsed.as_secs() * 1_000 + u64::from(elapsed.subsec_nanos()) / 1_000_000;
  // Compute the equivalent sample per pixel for a one minute computation budget
  // TODO

  println!("Done in {}ms, saving the picture", render_time_ms as isize);
  fb.normalize();
  match fb.write_ppm("rendering.ppm") {
    Ok(_) => {}
    Err(_) => {
      println!("Failed saving the picture");
    }
  }
}
