let
  sysPkg = import <nixpkgs> { };
  releasedPkgs = sysPkg.fetchFromGitHub {
    owner = "NixOS";
    repo = "nixpkgs";
    rev = "21.05";
    sha256 = "1ckzhh24mgz6jd1xhfgx0i9mijk6xjqxwsshnvq789xsavrmsc36";
  };
  stdenv = released_pkgs.stdenv;
  released_pkgs = import releasedPkgs {};

in stdenv.mkDerivation {
  name = "env";
  buildInputs = [ released_pkgs.gnumake
                  released_pkgs.arduino-cli
                  released_pkgs.wget
                ];
  shellHook = ''
            source .env
  '';

}
