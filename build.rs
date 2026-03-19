fn main() {
    cc::Build::new()
        .file("c/ble_transport.c")
        .include("c")
        .compile("ble_transport");

    println!("cargo:rustc-link-lib=bluetooth");
}
