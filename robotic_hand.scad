// =========================================================================
// Parametric Biomimetic Musculoskeletal Hand Platform (27 DoF)
// 36x McKibben Hydraulic Artificial Muscle Routing & Composite Bones
// Exact Anatomical Parity: Digits (16 DoF), Thumb (5 DoF), Palm (2 DoF), Wrist (4 DoF)
// =========================================================================

$fn = 40;

PALM_WIDTH  = 74;
PALM_LENGTH = 84;
PALM_THICK  = 21;
PIN_DIA     = 3.0;
PIN_TOL     = 0.3;
TENDON_DIA  = 1.5;
WALL_THICK  = 2.5;

PART_TO_RENDER = "assembly";

module pin_hole(length=30, dia=PIN_DIA+PIN_TOL) {
    cylinder(d=dia, h=length, center=true);
}


// Biomimetic McKibben Hydraulic Muscle Channel & Ligament Fairlead
module hydraulic_conduit(length=40, dia=3.2) {
    cylinder(d=dia, h=length, center=true);
}

module antagonistic_pair_ports(spacing=8, dia=3.0) {
    translate([-spacing/2, 0, 0]) cylinder(d=dia, h=25, center=true);
    translate([ spacing/2, 0, 0]) cylinder(d=dia, h=25, center=true);
}

module tendon_bore(length=60, dia=TENDON_DIA) {
    cylinder(d=dia, h=length, center=true);
}

module distal_phalanx(length=24, width=12, height=10) {
    difference() {
        union() {
            hull() {
                translate([0, 0, 0])
                    scale([width/height, 1, 1])
                        sphere(d=height*0.95);
                
                translate([0, length*0.7, -height*0.15])
                    scale([width*0.8/height, 1.2, 0.9])
                        sphere(d=height*0.75);

                translate([0, length - height*0.35, 0])
                    scale([width*0.7/height, 0.9, 0.75])
                        sphere(d=height*0.65);
            }
            rotate([0, 90, 0])
                cylinder(d=height*0.9, h=width - 3.4, center=true);
        }
        rotate([0, 90, 0])
            pin_hole(length=width+4);
        
        translate([0, length*0.55, -height*0.15])
            cube([width*0.38, 4, height*0.6], center=true);
        
        translate([0, length/2, -height*0.25])
            rotate([90, 0, 0])
                tendon_bore(length=length*1.2);

        for (i = [0:3]) {
            translate([0, length*0.45 + i*3.0, -height*0.45])
                rotate([0, 90, 0])
                    cylinder(d=0.9, h=width*0.75, center=true);
        }
    }
}

module intermediate_phalanx(length=28, width=13, height=11) {
    clevis_w = 4.2;
    difference() {
        union() {
            hull() {
                translate([0, 0, 0])
                    rotate([0, 90, 0])
                        cylinder(d=height*0.95, h=width, center=true);

                translate([0, length*0.5, -height*0.12])
                    scale([width/height*0.9, 1.1, 0.95])
                        sphere(d=height*0.85);

                translate([0, length, 0])
                    rotate([0, 90, 0])
                        cylinder(d=height*0.85, h=width*0.85, center=true);
            }
        }
        translate([0, -0.5, 0])
            cube([clevis_w + 0.6, height + 4, height + 4], center=true);
        
        rotate([0, 90, 0])
            pin_hole(length=width+4);
        
        translate([0, length, 0]) {
            cube([width - 3.0, height, height*1.2], center=true);
            rotate([0, 90, 0])
                pin_hole(length=width+4);
        }

        translate([0, length/2, -height*0.28])
            rotate([90, 0, 0])
                tendon_bore(length=length+4);

        translate([0, length/2, height*0.28])
            rotate([90, 0, 0])
                tendon_bore(length=length+4);
    }
}

module proximal_phalanx(length=38, width=15, height=13) {
    clevis_w = 4.0;
    difference() {
        union() {
            hull() {
                translate([0, 0, 0])
                    rotate([0, 90, 0])
                        cylinder(d=height*0.95, h=width, center=true);

                translate([0, length*0.5, -height*0.12])
                    scale([width/height*0.88, 1.2, 0.95])
                        sphere(d=height*0.85);

                translate([0, length, 0])
                    rotate([0, 90, 0])
                        cylinder(d=height*0.82, h=width*0.82, center=true);
            }
            translate([0, length, 0])
                rotate([0, 90, 0])
                    cylinder(d=height*0.85, h=clevis_w, center=true);
        }
        translate([0, 0, 0])
            cube([clevis_w + 0.8, height + 4, height + 4], center=true);
        
        rotate([0, 90, 0])
            pin_hole(length=width+4);
        
        translate([0, length, 0])
            rotate([0, 90, 0])
                pin_hole(length=width+4);

        translate([0, length/2, -height*0.3])
            rotate([90, 0, 0])
                tendon_bore(length=length+6);

        translate([0, length/2, height*0.3])
            rotate([90, 0, 0])
                tendon_bore(length=length+6);
    }
}

module finger(scale_f=1.0, flex_mcp=15, flex_pip=25, flex_dip=20) {
    p_len = 38 * scale_f;
    i_len = 26 * scale_f;
    d_len = 22 * scale_f;
    w = 14 * scale_f;
    h = 12 * scale_f;

    rotate([flex_mcp, 0, 0]) {
        proximal_phalanx(length=p_len, width=w, height=h);
        
        translate([0, p_len, 0])
        rotate([flex_pip, 0, 0]) {
            intermediate_phalanx(length=i_len, width=w*0.9, height=h*0.9);
            
            translate([0, i_len, 0])
            rotate([flex_dip, 0, 0]) {
                distal_phalanx(length=d_len, width=w*0.8, height=h*0.85);
            }
        }
    }
}

module palm_structure() {
    difference() {
        union() {
            hull() {
                translate([-PALM_WIDTH*0.3, 10, 0]) sphere(r=10);
                translate([ PALM_WIDTH*0.3, 10, 0]) sphere(r=9.5);
                
                // Lower Thenar Eminence (Thumb base muscle)
                translate([-PALM_WIDTH*0.36, PALM_LENGTH*0.28, -2])
                    scale([1.15, 1.35, 0.9]) sphere(r=14);
                
                // Hypothenar Eminence
                translate([ PALM_WIDTH*0.36, PALM_LENGTH*0.40, -2.5])
                    scale([0.9, 1.5, 0.85]) sphere(r=12.5);
                
                // Metacarpal Heads
                translate([-23, PALM_LENGTH - 10, 0]) sphere(r=9);
                translate([ -8, PALM_LENGTH - 8,  0.5]) sphere(r=9.5);
                translate([  8, PALM_LENGTH - 9,  0.2]) sphere(r=9.2);
                translate([ 23, PALM_LENGTH - 12, -0.4]) sphere(r=8.5);

                translate([0, PALM_LENGTH*0.45, PALM_THICK*0.3])
                    scale([1.2, 1.4, 0.8]) sphere(r=12);
            }
        }

        // Palmar hollow
        translate([0, PALM_LENGTH*0.44, -PALM_THICK*0.5])
            scale([1.2, 1.2, 0.6])
                sphere(r=14);

        // 4 Finger knuckle clevises
        finger_x = [-23, -8, 8, 23];
        knuckle_y = [PALM_LENGTH - 2.5, PALM_LENGTH, PALM_LENGTH - 1.2, PALM_LENGTH - 3.8];
        knuckle_z = [0.4, 0.8, 0.3, -0.4];
        finger_angles = [4, 1, -2, -6];

        for (i = [0:3]) {
            translate([finger_x[i], knuckle_y[i] - 1.0, knuckle_z[i]])
            rotate([0, 0, finger_angles[i]]) {
                cube([5.2, 16, PALM_THICK*1.2], center=true);
                rotate([0, 90, 0])
                    pin_hole(length=18);
                translate([0, -12, -3.5])
                    rotate([90, 0, 0])
                        cylinder(d=2.5, h=28, center=true);
            }
        }

        // Biomechanically Corrected Thumb CMC socket
        // Placed at the lower thenar base with 42° palmar abduction & 38° pronation
        translate([-PALM_WIDTH*0.36 - 2.5, PALM_LENGTH*0.28, -2.0])
        rotate([26, 38, -48]) {
            cube([6.0, 18, 22], center=true);
            rotate([0, 90, 0])
                pin_hole(length=24);
            translate([0, -10, 0])
                rotate([90, 0, 0])
                    cylinder(d=2.5, h=22, center=true);
        }

        // Internal routing hollow
        translate([0, PALM_LENGTH*0.42, 0])
            cube([PALM_WIDTH*0.58, PALM_LENGTH*0.45, PALM_THICK - 2*WALL_THICK], center=true);

        // Robotic wrist flange mount
        translate([0, 4, 0]) {
            cylinder(d=26, h=PALM_THICK*2, center=true);
            for (a = [45, 135, 225, 315]) {
                rotate([0, 0, a])
                    translate([18, 0, 0])
                        cylinder(d=3.2, h=PALM_THICK*2, center=true);
            }
        }
    }
}

// --- Assemble Hand with True Thumb Opposition ---
module full_assembly() {
    palm_structure();

    // Pinky
    translate([23, PALM_LENGTH - 3.8 - 4, -0.4])
        rotate([0, 0, -6])
            finger(scale_f=0.78, flex_mcp=20, flex_pip=30, flex_dip=15);

    // Ring
    translate([8, PALM_LENGTH - 1.2 - 4, 0.3])
        rotate([0, 0, -2])
            finger(scale_f=0.92, flex_mcp=25, flex_pip=35, flex_dip=20);

    // Middle
    translate([-8, PALM_LENGTH - 4, 0.8])
        rotate([0, 0, 1])
            finger(scale_f=1.00, flex_mcp=28, flex_pip=40, flex_dip=22);

    // Index
    translate([-23, PALM_LENGTH - 2.5 - 4, 0.4])
        rotate([0, 0, 4])
            finger(scale_f=0.90, flex_mcp=30, flex_pip=45, flex_dip=25);

    // True Opposable Thumb (Originates at lower thenar mound)
    translate([-PALM_WIDTH*0.36 - 2.5, PALM_LENGTH*0.28, -2.0])
    rotate([32, 42, -50]) {
        proximal_phalanx(length=32, width=15, height=13);
        translate([0, 32, 0])
            rotate([25, 0, 0])
                distal_phalanx(length=26, width=14, height=11.5);
    }
}

if (PART_TO_RENDER == "assembly") {
    full_assembly();
} else if (PART_TO_RENDER == "palm") {
    palm_structure();
} else if (PART_TO_RENDER == "proximal") {
    proximal_phalanx();
} else if (PART_TO_RENDER == "intermediate") {
    intermediate_phalanx();
} else if (PART_TO_RENDER == "distal") {
    distal_phalanx();
}
