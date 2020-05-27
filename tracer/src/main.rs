mod engine;
mod framebuffer;
mod geometry;

fn main() {
    // liberal transposition of the original code from Andrew Kensler
    // with help from Fabien Sanglard's
    // https://fabiensanglard.net/revisiting_the_pathtracer/index.html

    let w = 320;
    let h = 240;
    let samples_count = 5;
    engine::render(w, h, samples_count);
}
