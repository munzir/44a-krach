# 44a-krach
C++ interface for ach communication on krang

## Dependencies

- `python-dev`, `libboost-python-dev`

      sudo apt install python-dev libboost-python-dev

- [37-somatic](https://github.gatech.edu/WholeBodyControlAttempt1/37-somatic)
 Install the repo.

### Optional Dependency

For simulation:

- [41-krang-sim-ach](https://github.gatech.edu/WholeBodyControlAttempt1/41-krang-sim-ach)
 Install the repository.

## Installation

    git clone https://github.gatech.edu/WholeBodyControlAttempt1/44a-krach
    cd 44a-krach
    mkdir build
    cd build
    cmake ..
    sudo make install

## Uninstall
 To remove system files by the installation of this repo.

    sudo make uninstall

## Build and Run

To compile

    mkdir build
    cd build
    cmake ..
    make

Lauch simulation using [41-krang-sim-ach](https://github.gatech.edu/WholeBodyControlAttempt1/41-krang-sim-ach), then

    python example.py
