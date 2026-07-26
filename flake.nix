{
  description = "ORB_SLAM3 with Web Visualizer Nix environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          config.allowUnfree = true;
        };

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

          buildInputs = with pkgs; [
            boost
            eigen
            opencv4
            openssl
            glew
            libGL
            libx11
            pyEnv
          ];

          shellHook = ''
            export PLAYWRIGHT_BROWSERS_PATH=${pkgs.playwright-driver.browsers}
            export PLAYWRIGHT_SKIP_BROWSER_DOWNLOAD=1
            echo "ORB_SLAM3 Web Visualizer Nix environment loaded."
          '';
        };
      });
}
