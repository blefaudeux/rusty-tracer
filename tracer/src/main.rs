mod engine;
mod framebuffer;
mod geometry;

fn main() {
    // liberal transposition of the original code from Andrew Kensler
    // with help from Fabien Sanglard's
    // https://fabiensanglard.net/revisiting_the_pathtracer/index.html

    let w = 960;
    let h = 540;
    let samples_count = 20;
    engine::render(w, h, samples_count);
}
