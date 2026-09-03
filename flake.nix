{
  description = "ORB_SLAM3 with Web Visualizer Nix environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    # Pangolin was removed from nixpkgs-unstable on 2025-11-17. It is still
    # shipped (v0.9.1) in the nixos-25.05 release, so we pull just the native
    # viewer dependency from there while keeping unstable as the primary
    # toolchain (unstable still provides python3Packages.rosbags etc.).
    nixpkgs-pango.url = "github:NixOS/nixpkgs/nixos-25.05";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, nixpkgs-pango, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          config.allowUnfree = true;
        };

        # Native Pangolin viewer (optional). Sourced from nixos-25.05 because it
        # has been dropped from unstable. Referenced outside any `with pkgs`
        # scope so it never resolves to the throwing `pkgs.pangolin` alias.
        pangolin = (import nixpkgs-pango {
          inherit system;
          config.allowUnfree = true;
        }).pangolin;

        pyEnv = pkgs.python3.withPackages (ps: with ps; [
          rosbags
          opencv4
          numpy
          playwright
          requests
          websockets
        ]);

      in {
        devShells.default = pkgs.mkShell {
          name = "orbslam3-web-dev";

          nativeBuildInputs = with pkgs; [
            cmake
            pkg-config
            gcc
            gnumake
            nodejs
            playwright-driver.browsers
          ];

          buildInputs = (with pkgs; [
            boost
            eigen
            gtest
            (opencv4.override { enableGtk3 = true; })
            openssl
            glew
            libGL
            libx11
            librealsense
            pyEnv
          ]) ++ [
            # Native Pangolin viewer dependency (see comment above).
            pangolin
          ];

          shellHook = ''
            export PLAYWRIGHT_BROWSERS_PATH=${pkgs.playwright-driver.browsers}
            export PLAYWRIGHT_SKIP_BROWSER_DOWNLOAD=1
            # nixpkgs ships CMake 4.x, which refuses the very old
            # `cmake_minimum_required(VERSION 2.8)` used by ORB-SLAM3 and its
            # bundled Thirdparty (DBoW2/g2o/Sophus). This compatibility shim lets
            # those configure steps run unmodified (equivalent to passing
            # -DCMAKE_POLICY_VERSION_MINIMUM=3.5 to every cmake invocation).
            export CMAKE_POLICY_VERSION_MINIMUM=3.5
            echo "ORB_SLAM3 Web Visualizer Nix environment loaded."
            echo "Pangolin native viewer available (${pangolin.pname}-${pangolin.version})."
          '';
        };
      });
}
