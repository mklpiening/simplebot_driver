$fn=32;

rotate([90, 0, 0]) {
    cylinder(h = 0.01, d = 0.049);
    translate([0, 0, -0.001]) {
        cylinder(h = 0.012, d = 0.029);
    }

    translate([0, 0, -0.006]) {
        difference() {
            union() {
                cylinder(h = 0.005, d = 0.027);
                translate([0, 0, -0.004]) {
                    cylinder(h = 0.004, d = 0.011);
                }
            }
            translate([0, 0, -0.01]) {
                cylinder(h = 0.02, d = 0.006);
            }
        }
    }
}