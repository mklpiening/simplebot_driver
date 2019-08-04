screwOffset = 0.035;
screwSideOffset = 0.055;
height = 0.05;
sideLength = 0.18;

$fn=32;

module motor(length, diameter, shaftLength, shaftDiameter) {
    rotate([0, 90, 0]) {
        cylinder(d=diameter, h=length);
        translate([diameter / 2 - 0.01, 0, -shaftLength -0.005]) {
            cylinder(d=shaftDiameter, h=shaftLength + 0.005);
            translate([0, 0, shaftLength]) {
                cylinder(d=shaftDiameter * 2, h=0.005);
            }
        }
    }
    translate([0.001, 0, diameter / 4]) {
        cube([0.001, diameter, diameter * 0.5], true);
    }
    translate([length * 0.35 + 0.001, 0, diameter * 0.5]) {
        cube([length * 0.7, diameter, 0.001], true);
    }
    translate([0, 0, diameter * 0.5]) {
        translate([length * 0.7 - 0.005, diameter * 0.5 - 0.005, 0]) {
            cylinder(d=0.004, h=0.002);
        }
        translate([length * 0.7 - 0.005, -(diameter * 0.5 - 0.005), 0]) {
            cylinder(d=0.004, h=0.002);
        }
        translate([0.005, diameter * 0.5 - 0.005, 0]) {
            cylinder(d=0.004, h=0.002);
        }
        translate([0.005, -(diameter * 0.5 - 0.005), 0]) {
            cylinder(d=0.004, h=0.002);
        }
    }
}

module baseplate(side, strength, cornerCutoutDiagonal) {
    difference() {
        cube([side, side, strength]);
        translate([side, 0, 0]) {
            rotate([0, 0, 45]) {
                cube([cornerCutoutDiagonal, cornerCutoutDiagonal, strength * 2 + 1], true);
            }
        }
        translate([side, side, 0]) {
            rotate([0, 0, 45]) {
                cube([cornerCutoutDiagonal, cornerCutoutDiagonal, strength * 2 + 1], true);
            }
        }
        translate([0, 0, 0.1]) {
            rotate([0, 0, 45]) {
                cube([cornerCutoutDiagonal, cornerCutoutDiagonal, strength * 2 + 1], true);
            }
        }
        translate([0, side, 0]) {
            rotate([0, 0, 45]) {
                cube([cornerCutoutDiagonal, cornerCutoutDiagonal, strength * 2 + 1], true);
            }
        }
    }
}

rotate([0, 0, 90]) {
    translate([-sideLength/2, -sideLength/2, 0]) {
        baseplate(sideLength, 0.005, 0.044);

        translate([screwSideOffset, screwOffset, 0.005]) {
            cube([0.004, 0.004, height]);
        }
        translate([sideLength - screwSideOffset, screwOffset, 0.005]) {
            cube([0.004, 0.004, height]);
        }
        translate([screwSideOffset, sideLength - screwOffset, 0.005]) {
            cube([0.004, 0.004, height]);
        }
        translate([sideLength - screwSideOffset, sideLength - screwOffset, 0.005]) {
            cube([0.004, 0.004, height]);
        }

        difference() {
            translate([0, 0, height]) {
                baseplate(sideLength, 0.005, 0.045);
            }
            translate([0.009999, sideLength / 2, 0.0451]) {
                cube([0.02, 0.03, 0.08], true);
            }
        }

        translate([0.01, sideLength / 2, 0.045]) {
            difference() {
                cube([0.02, 0.03, 0.08], true);
                cube([0.01, 0.02, 0.081], true);
            }
        }

        translate([0, 0, -0.0195]) { 
            translate([0.027, 0.055, 0]) { 
                motor(0.055, 0.035, 0.013, 0.004);
            }
            translate([0.027, sideLength - 0.055, 0]) { 
                motor(0.055, 0.035, 0.013, 0.004);
            }
            translate([sideLength, 0, 0]) {
                mirror() {
                    translate([0.027, 0.055, 0]) { 
                        motor(0.055, 0.035, 0.013, 0.005);
                    }
                    translate([0.027, sideLength - 0.055, 0]) { 
                        motor(0.055, 0.035, 0.013, 0.005);
                    }
                }
            }
        }
    }
}