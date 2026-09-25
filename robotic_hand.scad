// =========================================================================
// Parametric Anthropomorphic 5-Finger Robotic Hand CAD Model
// Human-Like Organic / Ergonomically Rounded Phalanxes & Palmar Pads
// Compatible with OpenSCAD (exportable to STL/3MF/STEP)
// Suitable for 3D Printing & Tendon-Driven Actuation
// =========================================================================

$fn = 40; // Resolution

// Global Dimensions (mm)
PALM_WIDTH  = 72;
PALM_LENGTH = 82;
PALM_THICK  = 20;
PIN_DIA     = 3.0;   // M3 bolt or 3mm stainless dowel pin
PIN_TOL     = 0.3;   // Pin clearance
TENDON_DIA  = 1.5;   // Bowden / Dyneema tendon guide channel
WALL_THICK  = 2.5;

PART_TO_RENDER = "assembly"; // "assembly", "palm", "proximal", "intermediate", "distal"

// --- Module: Pin Hole ---
module pin_hole(length=30, dia=PIN_DIA+PIN_TOL) {
    cylinder(d=dia, h=length, center=true);
}

// --- Module: Tendon Channel ---
module tendon_bore(length=60, dia=TENDON_DIA) {
    cylinder(d=dia, h=length, center=true);
}

// --- Module: Distal Phalanx (Curved Human-like Fingertip) ---
module distal_phalanx(length=24, width=12, height=10) {
    difference() {
        union() {
            // Organic hull of curved cross-sections
            hull() {
                // Base knuckle sphere
                translate([0, 0, 0])
                    scale([width/height, 1, 1])
                        sphere(d=height*0.95);
                
                // Ergonomic palmar pulp cushion
                translate([0, length*0.7, -height*0.15])
                    scale([width*0.8/height, 1.2, 0.9])
                        sphere(d=height*0.75);

                // Fingertip apex
                translate([0, length - height*0.35, 0])
                    scale([width*0.7/height, 0.9, 0.75])
                        sphere(d=height*0.65);
            }
            // Joint knuckle pivot boss
            rotate([0, 90, 0])
                cylinder(d=height*0.9, h=width - 3.4, center=true);
        }
        // Hinge pin hole (DIP Joint)
        rotate([0, 90, 0])
            pin_hole(length=width+4);
        
        // Tendon termination pocket
        translate([0, length*0.55, -height*0.15])
            cube([width*0.38, 4, height*0.6], center=true);
        
        // Longitudinal flexor tendon bore
        translate([0, length/2, -height*0.25])
            rotate([90, 0, 0])
                tendon_bore(length=length*1.2);

        // Tactile fingerprint ridge grooves
        for (i = [0:3]) {
            translate([0, length*0.45 + i*3.0, -height*0.45])
                rotate([0, 90, 0])
                    cylinder(d=0.9, h=width*0.75, center=true);
        }
    }
}

// --- Module: Intermediate Phalanx (Rounded Segment) ---
module intermediate_phalanx(length=28, width=13, height=11) {
    clevis_w = 4.2;
    difference() {
        union() {
            hull() {
                // Proximal rounded head
                translate([0, 0, 0])
                    rotate([0, 90, 0])
                        cylinder(d=height*0.95, h=width, center=true);

                // Anatomical palmar fleshy curve
                translate([0, length*0.5, -height*0.12])
                    scale([width/height*0.9, 1.1, 0.95])
                        sphere(d=height*0.85);

                // Distal rounded head
                translate([0, length, 0])
                    rotate([0, 90, 0])
                        cylinder(d=height*0.85, h=width*0.85, center=true);
            }
        }
        // Proximal Clevis Slot
        translate([0, -0.5, 0])
            cube([clevis_w + 0.6, height + 4, height + 4], center=true);
        
        // Proximal Pin Hole (PIP)
        rotate([0, 90, 0])
            pin_hole(length=width+4);
        
        // Distal Clevis Slot
        translate([0, length, 0]) {
            cube([width - 3.0, height, height*1.2], center=true);
            rotate([0, 90, 0])
                pin_hole(length=width+4);
        }

        // Flexor tendon channel
        translate([0, length/2, -height*0.28])
            rotate([90, 0, 0])
                tendon_bore(length=length+4);

        // Extensor tendon channel
        translate([0, length/2, height*0.28])
            rotate([90, 0, 0])
                tendon_bore(length=length+4);
    }
}

// --- Module: Proximal Phalanx (Curved Base Bone) ---
module proximal_phalanx(length=38, width=15, height=13) {
    clevis_w = 4.0;
    difference() {
        union() {
            hull() {
                // Metacarpal knuckle rounded head
                translate([0, 0, 0])
                    rotate([0, 90, 0])
                        cylinder(d=height*0.95, h=width, center=true);

                // Fleshy palmar curve
                translate([0, length*0.5, -height*0.12])
                    scale([width/height*0.88, 1.2, 0.95])
                        sphere(d=height*0.85);

                // Distal neck
                translate([0, length, 0])
                    rotate([0, 90, 0])
                        cylinder(d=height*0.82, h=width*0.82, center=true);
            }
            // Distal central tongue
            translate([0, length, 0])
                rotate([0, 90, 0])
                    cylinder(d=height*0.85, h=clevis_w, center=true);
        }
        // Base clevis slot
        translate([0, 0, 0])
            cube([clevis_w + 0.8, height + 4, height + 4], center=true);
        
        // Pin holes
        rotate([0, 90, 0])
            pin_hole(length=width+4);
        
        translate([0, length, 0])
            rotate([0, 90, 0])
                pin_hole(length=width+4);

        // Tendon conduits
        translate([0, length/2, -height*0.3])
            rotate([90, 0, 0])
                tendon_bore(length=length+6);

        translate([0, length/2, height*0.3])
            rotate([90, 0, 0])
                tendon_bore(length=length+6);
    }
}

// --- Articulated Finger ---
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

// --- Curved Ergonomic Palm ---
module palm_structure() {
    difference() {
        union() {
            // Smooth natural palm chassis
            hull() {
                translate([-PALM_WIDTH/2 + 10, 8, 0]) sphere(r=10);
                translate([ PALM_WIDTH/2 - 10, 8, 0]) sphere(r=10);
                translate([-PALM_WIDTH/2 + 12, PALM_LENGTH - 10, 0]) sphere(r=11);
                translate([ PALM_WIDTH/2 - 10, PALM_LENGTH - 10, 0]) sphere(r=9.5);
            }
            // Thenar eminence (thumb base muscle pad)
            translate([-PALM_WIDTH/2 - 2, PALM_LENGTH*0.35, -2])
                rotate([15, 30, -35])
                    scale([1, 1.2, 0.9])
                        sphere(d=22);
        }

        // Knuckle Clevis Mounts
        finger_x = [-23, -8, 8, 23];
        finger_angles = [4, 1, -2, -6];
        for (i = [0:3]) {
            translate([finger_x[i], PALM_LENGTH - 2, 0])
            rotate([0, 0, finger_angles[i]]) {
                cube([5.2, 16, PALM_THICK*1.2], center=true);
                rotate([0, 90, 0])
                    pin_hole(length=18);
                translate([0, -10, -4])
                    rotate([90, 0, 0])
                        cylinder(d=2.5, h=25, center=true);
            }
        }

        // Thumb Socket
        translate([-PALM_WIDTH/2 - 2, PALM_LENGTH*0.35, -2])
        rotate([15, 30, -35]) {
            cube([6.0, 16, 20], center=true);
            rotate([0, 90, 0])
                pin_hole(length=22);
            translate([0, -8, 0])
                rotate([90, 0, 0])
                    cylinder(d=2.5, h=20, center=true);
        }

        // Hollow cavity
        translate([0, PALM_LENGTH*0.4, 0])
            cube([PALM_WIDTH*0.65, PALM_LENGTH*0.52, PALM_THICK - 2*WALL_THICK], center=true);

        // Wrist robotic flange mount
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

// --- Assemble Entire Hand ---
module full_assembly() {
    palm_structure();

    // Pinky
    translate([23, PALM_LENGTH - 2, 0])
        rotate([0, 0, -6])
            finger(scale_f=0.78, flex_mcp=20, flex_pip=30, flex_dip=15);

    // Ring
    translate([8, PALM_LENGTH - 1, 0])
        rotate([0, 0, -2])
            finger(scale_f=0.92, flex_mcp=25, flex_pip=35, flex_dip=20);

    // Middle
    translate([-8, PALM_LENGTH, 0])
        rotate([0, 0, 1])
            finger(scale_f=1.00, flex_mcp=28, flex_pip=40, flex_dip=22);

    // Index
    translate([-23, PALM_LENGTH - 2, 0])
        rotate([0, 0, 4])
            finger(scale_f=0.90, flex_mcp=30, flex_pip=45, flex_dip=25);

    // Opposable Thumb
    translate([-PALM_WIDTH/2 - 2, PALM_LENGTH*0.35, -2])
    rotate([25, 35, -40]) {
        proximal_phalanx(length=32, width=15, height=13);
        translate([0, 32, 0])
            rotate([30, 0, 0])
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
