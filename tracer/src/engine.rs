
use crate::geometry::Vec3f;
use crate::geometry::box_test;
use crate::geometry::Ray;
use crate::geometry::min;

enum HitType {
    HitNone,
    HitLetter,
    HitWall,
    HitSun
}


// 15*4+1
static letters : Vec<u8> =  concat!(           // 15 two points lines
    "5O5_", "5W9W", "5_9_",             // P (without curve)
    "AOEO", "COC_", "A_E_",             // I
    "IOQ_", "I_QO",                     // X
    "UOY_", "Y_]O", "WW[W",             // A
    "aOa_", "aWeW", "a_e_", "cWiO").as_bytes().to_vec();    // R (without curve)


// Sample the world using Signed Distance Fields.
fn  sample_world(position: Vec3f) -> (f64, HitType) {
  let mut distance : f64 = 1e9;
  let mut hitType: HitType = HitType::HitNone;

  let mut f = position; // Flattened position (z=0)
  f.z = 0.;

  for i in 0..letters.len()/4 {

    let begin = Vec3f{
        x: letters[4*i]  as f64 - 79., 
        y: letters[4*i + 1]  as f64 - 79., 
        z:0.
    }.scaled(0.5);

    let end = Vec3f{
        x : letters[i + 2] as f64- 79., 
        y:letters[4*i + 3] as f64 - 79.,
        z:0.
    }.scaled(0.5) - begin;

    let o = f - begin + end.scaled(
            min(
                -min((begin - f).dot(end) / end.squared_norm(), 0.),
                1.));

    distance = min(distance, o.squared_norm()); // compare squared distance.
  }

  // Get real distance, not square distance, once all the comparisons are done
  distance = distance.sqrt();

  // Two curves (for P and R in PixaR) with hard-coded locations.
  let curves : [Vec3f; 2] = [Vec3f{x:11., y:6., z:0.}, Vec3f{x:-11., y:6., z:0.}];

  for curve in curves.iter() {
    let o = f - *curve;

    distance = min(distance,
                   if o.x > 0. {(o.norm() - 2.).abs()} else {
                       o.y += o.y;
                       if o.y > 0. {-2.} else {2.}}
                    //    , o.norm()
               );
  }

  distance = (distance.powi(8) + position.z.powi(8)).powf(0.125) - 0.5;
  hitType = HitType::HitLetter;

  let roomDist = min(// min(A,B) = Union with Constructive solid geometry
               //-min carves an empty space
                -min(
                    // Lower room
                    box_test(position, Vec3f{x:-30., y:-0.5, z:-30.}, Vec3f{x:30., y:18., z:30.}),
                    
                    // Upper room
                    box_test(position, Vec3f{x:-25., y:17., z:-25.}, Vec3f{x:25., y:20., z:25.})
                ),
                box_test( // Ceiling "planks" spaced 8 units apart.
                Vec3f{x: position.x.abs() % 8.,
                      y:position.y,
                      z:position.z},
                      Vec3f{x:1.5, y:18.5, z:-25.},
                      Vec3f{x:6.5, y:20., z:25.})
                
  );

  if roomDist < distance {
      distance = roomDist;
      hitType = HitType::HitWall;
    }

// Everything above 19.9 is light source.
  let sun = 19.9 - position.y ; 
  if sun < distance {
      distance = sun;
       hitType = HitType::HitSun;
  }

  return (distance, hitType);
}

// Perform signed sphere marching
// Returns hitType 0, 1, 2, or 3 and update hit position/normal
fn ray_marching( ray: Ray) -> (HitType, f64, f64) {
  let mut hitType = HitType::HitNone;
  let mut noHitCount = 0;
  let hitPos : Vec3f = Vec3f::ones();
  let hitNorm : Vec3f = Vec3f::ones();

  let mut shortest_distance = 1e9; // distance from closest object in world.

  // Signed distance marching
  let mut total_distance = 0.;
  while total_distance < 100. {
    hitPos = ray.orig + ray.dir.scaled(total_distance);
    
    (shortest_distance, hitType) = sample_world(hitPos);
    noHitCount += 1;

    if (shortest_distance < 0.01) || noHitCount > 99 {
        hitNorm = Vec3f{
            x: sample_world(hitPos + Vec3f{x: 0.01, y:0., z:0.}, noHitCount)[0] - shortest_distance,
            y: sample_world(hitPos + Vec3f{x:0., y:0.01, z:0.}, noHitCount)[0] - shortest_distance,
            z: sample_world(hitPos + Vec3f{x:0., y:0., z:0.01}, noHitCount)[0] - shortest_distance
        }.normalized();
    }
  }

  return (hitType, hitPos, hitNorm);
}

fn Trace(ray: Ray) -> Vec3f {
  Vec sampledPosition, normal, color, attenuation = 1;
  Vec lightDirection(!Vec(.6, .6, 1)); // Directional light

  for (int bounceCount = 3; bounceCount--;) {
    int hitType = RayMarching(origin, direction, sampledPosition, normal);
    if (hitType == HIT_NONE) break; // No hit. This is over, return color.
    if (hitType == HIT_LETTER) { // Specular bounce on a letter. No color acc.
      direction = direction + normal * ( normal % direction * -2);
      origin = sampledPosition + direction * 0.1;
      attenuation = attenuation * 0.2; // Attenuation via distance traveled.
    }
    if (hitType == HIT_WALL) { // Wall hit uses color yellow?
      float incidence = normal % lightDirection;
      float p = 6.283185 * randomVal();
      float c = randomVal();
      float s = sqrtf(1 - c);
      float g = normal.z < 0 ? -1 : 1;
      float u = -1 / (g + normal.z);
      float v = normal.x * normal.y * u;
      direction = Vec(v,
                      g + normal.y * normal.y * u,
                      -normal.y) * (cosf(p) * s)
                  +
                  Vec(1 + g * normal.x * normal.x * u,
                      g * v,
                      -g * normal.x) * (sinf(p) * s) + normal * sqrtf(c);
      origin = sampledPosition + direction * .1;
      attenuation = attenuation * 0.2;
      if (incidence > 0 &&
          RayMarching(sampledPosition + normal * .1,
                      lightDirection,
                      sampledPosition,
                      normal) == HIT_SUN)
        color = color + attenuation * Vec(500, 400, 100) * incidence;
    }
    if (hitType == HIT_SUN) { //
      color = color + attenuation * Vec(50, 80, 100); break; // Sun Color
    }
  }
  return color;
}
