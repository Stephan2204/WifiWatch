
$fn=48;
lid_file = "/mnt/data/ESP32-S3-ETH-display-lid-v2/ESP32-S3-ETH-lid-repaired-watertight.stl";

cx=8.2;
cy=38.2;
window=29.0;
dx=39.0;
dy=27.0;
boss_od=5.0;
pilot_d=1.6;
boss_h=3.6;
boss_z=22.799999999999997;

module lid() {
    import(lid_file, convexity=10);
}

module display_window() {
    translate([cx-window/2, cy-window/2, 20])
        cube([window, window, 15], center=false);
}

module boss(x,y) {
    difference() {
        translate([x,y,boss_z])
            cylinder(d=boss_od, h=boss_h);
        translate([x,y,boss_z-0.2])
            cylinder(d=pilot_d, h=boss_h+0.4);
    }
}

difference() {
    union() {
        difference() {
            lid();
            display_window();
        }
        boss(cx-dx/2, cy-dy/2);
        boss(cx+dx/2, cy-dy/2);
        boss(cx-dx/2, cy+dy/2);
        boss(cx+dx/2, cy+dy/2);
    }
    display_window();
}
