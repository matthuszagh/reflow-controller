{ pkgs ? (import <nixpkgs> {} // import <custompkgs> {}) }:
let local-python = (pkgs.python2.withPackages (ps: with ps; [
  pyserial
  matplotlib
])).override (args: {
  ignoreCollisions = true;
});
in
pkgs.mkShell {
  buildInputs = with pkgs; [
    local-python
    avrdude
  ] ++ (with pkgsCross; [
    avr.buildPackages.binutils
    avr.buildPackages.gcc7
  ]);
}
