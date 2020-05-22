mod geometry;
mod engine;
use geometry::Vec3f;

fn main() {
    // liberal transposition of the original code from Andrew Kensler
    // with help from Fabien Sanglard's 
    // https://fabiensanglard.net/revisiting_the_pathtracer/index.html

    let w = 960;
    let h = 540;
    let samplesCount = 1;

    let position = Vec3f{x:-22., y:5., z:25.};

    let goal = (Vec3f{x:-3., y:4., z:0.} -position).normalized();
    let left = Vec3f{x:goal.z, y:0., z:-goal.x}.normalized();
  
    // Cross-product to get the up vector
    let up = goal.cross(left);
    
    println!("Rendering {}x{}", w,h);
    
    // for (int y = h; y--;)
    //   for (int x = w; x--;) {
    //     Vec color;
    //     for (int p = samplesCount; p--;)
    //       color = color + Trace(position, !(goal + left * (x - w / 2 + randomVal()) + up * (y - h / 2 + randomVal())));
  
    //     // Reinhard tone mapping
    //     color = color * (1. / samplesCount) + 14. / 241;
    //     Vec o = color + 1;
    //     color = Vec(color.x / o.x, color.y / o.y, color.z / o.z) * 255;
    //     printf("%c%c%c", (int) color.x, (int) color.y, (int) color.z);
    //   }
}
