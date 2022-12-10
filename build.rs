fn main() {
    #[cfg(feature = "debugger")]
    println!("cargo:rustc-link-arg=-Tdefmt.x")
}
