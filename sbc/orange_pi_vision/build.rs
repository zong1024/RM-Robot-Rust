use std::env;
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=ORBBEC_SDK_V1_DIR");

    if env::var_os("CARGO_FEATURE_ORBBEC_SDK").is_none() {
        return;
    }

    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let target_arch = env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default();
    let sdk_root = match env::var_os("ORBBEC_SDK_V1_DIR") {
        Some(path) => PathBuf::from(path),
        None if target_arch == "x86_64" => {
            manifest_dir.join("third_party/orbbec_v1_10_18/sdk/OrbbecSDK_v1.10.18/SDK")
        }
        None => {
            panic!(
                "ORBBEC_SDK_V1_DIR is required for target arch {target_arch}. \
                 On Orange Pi AI Pro 8T, install an aarch64 OrbbecSDK v1 package and set \
                 ORBBEC_SDK_V1_DIR to its SDK directory containing include/ and lib/."
            );
        }
    };
    let sdk_lib = sdk_root.join("lib");

    if !sdk_lib.join("libOrbbecSDK.so").exists() {
        panic!(
            "libOrbbecSDK.so not found in {}. Set ORBBEC_SDK_V1_DIR to the OrbbecSDK v1 SDK directory that contains include/ and lib/ for the current target architecture ({target_arch}).",
            sdk_root.display(),
        );
    }

    println!("cargo:rustc-link-search=native={}", sdk_lib.display());
    println!("cargo:rustc-link-lib=dylib=OrbbecSDK");
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", sdk_lib.display());
}
