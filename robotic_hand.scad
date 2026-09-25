// =========================================================================
// Parametric Anthropomorphic 5-Finger Robotic Hand CAD Model
// Compatible with OpenSCAD (exportable to STL/3MF/AMF/CSG/DXF/SVG)
// Suitable for FDM/SLA 3D printing & Tendon-Driven Actuation
// =========================================================================

$fn = 40; // Mesh resolution

// Global Dimensions (mm)
PALM_WIDTH  = 72;
PALM_LENGTH = 82;
PALM_THICK  = 20;
PIN_DIA     = 3.0;   // M3 bolt or 3mm stainless dowel pin
PIN_TOL     = 0.3;   // Pin clearance
TENDON_DIA  = 1.5;   // Bowden / Dyneema tendon guide channel
WALL_THICK  = 2.5;

// Mode selection: "assembly", "palm", "proximal", "intermediate", "distal"
PART_TO_RENDER = "assembly";

// --- Module: Pin Hole ---
module pin_hole(length=30, dia=PIN_DIA+PIN_TOL) {
    cylinder(d=dia, h=length, center=true);
}

// --- Module: Tendon Channel ---
module tendon_bore(length=60, dia=TENDON_DIA) {
    cylinder(d=dia, h=length, center=true);
}

// --- Module: Distal Phalanx (Fingertip) ---
module distal_phalanx(length=24, width=12, height=10) {
    difference() {
        union() {
            // Main tapered body
            hull() {
                translate([0, 0, 0])
                    sphere(d=height);
                translate([0, length - width/2, 0])
                    sphere(d=height*0.75);
                translate([-width/2+height/2, 0, 0])
                    cylinder(d=height, h=1, center=true);
                translate([width/2-height/2, 0, 0])
                    cylinder(d=height, h=1, center=true);
            }
            // Joint knuckle knuckle base
            rotate([0, 90, 0])
                cylinder(d=height, h=width - 3.5, center=true);
        }
        // Hinge pin hole (DIP Joint)
        rotate([0, 90, 0])
            pin_hole(length=width+4);
        
        // Tendon termination pocket & anchor knot hole
        translate([0, length*0.6, -height*0.1])
            cube([width*0.4, 4, height*0.6], center=true);
        
        // Tendon guide hole running longitudinally
        translate([0, length/2, -height*0.25])
            rotate([90, 0, 0])
                tendon_bore(length=length*1.2);

        // Tactile grip grooves on pad
        for (i = [0:2]) {
            translate([0, length*0.4 + i*3.5, -height/2 + 0.5])
                cube([width*0.8, 1.2, 1], center=true);
        }
    }
}

// --- Module: Intermediate Phalanx ---
module intermediate_phalanx(length=28, width=13, height=11) {
    clevis_w = 4.2;
    difference() {
        union() {
            // Segment body
            hull() {
                translate([0, 0, 0])
                    rotate([0, 90, 0])
                        cylinder(d=height, h=width, center=true);
                translate([0, length, 0])
                    rotate([0, 90, 0])
                        cylinder(d=height*0.9, h=width*0.85, center=true);
            }
        }
        // Proximal Clevis Slot (receives proximal phalanx tongue)
        translate([0, -0.5, 0])
            cube([clevis_w + 0.6, height + 4, height + 4], center=true);
        
        // Proximal Pin Hole (PIP Joint)
        rotate([0, 90, 0])
            pin_hole(length=width+4);
        
        // Distal Clevis Slot (for distal tongue)
        translate([0, length, 0]) {
            cube([width - 3.0, height, height*1.2], center=true);
            rotate([0, 90, 0])
                pin_hole(length=width+4);
        }

        // Internal Tendon flexor & extensor channels
        translate([0, length/2, -height*0.28])
            rotate([90, 0, 0])
                tendon_bore(length=length+4);

        translate([0, length/2, height*0.28])
            rotate([90, 0, 0])
                tendon_bore(length=length+4);
    }
}

// --- Module: Proximal Phalanx ---
module proximal_phalanx(length=38, width=15, height=13) {
    clevis_w = 5.0;
    difference() {
        union() {
            // Main finger bone hull
            hull() {
                translate([0, 0, 0])
                    rotate([0, 90, 0])
                        cylinder(d=height, h=width, center=true);
                translate([0, length, 0])
                    rotate([0, 90, 0])
                        cylinder(d=height*0.85, h=width*0.85, center=true);
            }
            // Distal central tongue (inserts into intermediate clevis)
            translate([0, length, 0])
                rotate([0, 90, 0])
                    cylinder(d=height*0.85, h=clevis_w, center=true);
        }
        // MCP joint slot at base
        translate([0, 0, 0])
            cube([clevis_w + 0.8, height + 4, height + 4], center=true);
        
        // MCP pin hole
        rotate([0, 90, 0])
            pin_hole(length=width+4);
        
        // Distal pin hole (PIP)
        translate([0, length, 0])
            rotate([0, 90, 0])
                pin_hole(length=width+4);

        // Flexor tendon conduit
        translate([0, length/2, -height*0.3])
            rotate([90, 0, 0])
                tendon_bore(length=length+6);

        // Extensor tendon conduit
        translate([0, length/2, height*0.3])
            rotate([90, 0, 0])
                tendon_bore(length=length+6);
    }
}

// --- Complete Articulated Finger Module ---
module finger(scale_f=1.0, flex_mcp=15, flex_pip=25, flex_dip=20) {
    p_len = 38 * scale_f;
    i_len = 26 * scale_f;
    d_len = 22 * scale_f;
    w = 14 * scale_f;
    h = 12 * scale_f;

    // MCP
    rotate([flex_mcp, 0, 0]) {
        proximal_phalanx(length=p_len, width=w, height=h);
        
        // PIP
        translate([0, p_len, 0])
        rotate([flex_pip, 0, 0]) {
            intermediate_phalanx(length=i_len, width=w*0.9, height=h*0.9);
            
            // DIP
            translate([0, i_len, 0])
            rotate([flex_dip, 0, 0]) {
                distal_phalanx(length=d_len, width=w*0.8, height=h*0.85);
            }
        }
    }
}

// --- Module: Palm Base & Servo / Tendon Housing ---
module palm_structure() {
    difference() {
        union() {
            // Main palm anatomical chassis
            hull() {
                translate([-PALM_WIDTH/2 + 8, 0, 0]) cylinder(r=8, h=PALM_THICK, center=true);
                translate([ PALM_WIDTH/2 - 8, 0, 0]) cylinder(r=8, h=PALM_THICK, center=true);
                translate([-PALM_WIDTH/2 + 10, PALM_LENGTH - 10, 0]) cylinder(r=10, h=PALM_THICK*0.8, center=true);
                translate([ PALM_WIDTH/2 - 6, PALM_LENGTH - 10, 0]) cylinder(r=8, h=PALM_THICK*0.8, center=true);
            }
            // Thumb abductor mount boss
            translate([-PALM_WIDTH/2 - 2, PALM_LENGTH*0.35, -2])
                rotate([15, 30, -35])
                    cube([18, 22, 16], center=true);
        }

        // Finger Knuckle Knuckle Clevis Mounts (Index, Middle, Ring, Pinky)
        finger_x = [-23, -8, 8, 23];
        finger_angles = [4, 1, -2, -6];
        for (i = [0:3]) {
            translate([finger_x[i], PALM_LENGTH - 2, 0])
            rotate([0, 0, finger_angles[i]]) {
                cube([5.5, 16, PALM_THICK*1.2], center=true);
                rotate([0, 90, 0])
                    pin_hole(length=18);
                translate([0, -10, -4])
                    rotate([90, 0, 0])
                        cylinder(d=2.5, h=25, center=true);
            }
        }

        // Thumb CMC Mount Socket
        translate([-PALM_WIDTH/2 - 2, PALM_LENGTH*0.35, -2])
        rotate([15, 30, -35]) {
            cube([6.0, 16, 20], center=true);
            rotate([0, 90, 0])
                pin_hole(length=22);
            translate([0, -8, 0])
                rotate([90, 0, 0])
                    cylinder(d=2.5, h=20, center=true);
        }

        // Internal Cavity for Tendon routing
        translate([0, PALM_LENGTH*0.4, 0])
            cube([PALM_WIDTH*0.68, PALM_LENGTH*0.55, PALM_THICK - 2*WALL_THICK], center=true);

        // Wrist robotic flange mount (ISO pattern 4x M3)
        translate([0, -6, 0]) {
            cylinder(d=28, h=PALM_THICK*1.5, center=true);
            for (a = [45, 135, 225, 315]) {
                rotate([0, 0, a])
                    translate([18, 0, 0])
                        cylinder(d=3.2, h=PALM_THICK*1.5, center=true);
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

    // Thumb (Opposable orientation, 2 phalanges)
    translate([-PALM_WIDTH/2 - 2, PALM_LENGTH*0.35, -2])
    rotate([25, 35, -40]) {
        proximal_phalanx(length=32, width=15, height=13);
        translate([0, 32, 0])
            rotate([30, 0, 0])
                distal_phalanx(length=26, width=14, height=11.5);
    }
}

// --- Execution Selector ---
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
