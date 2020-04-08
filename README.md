[![Build Status](https://travis-ci.com/Ibujah/compactskel.svg?branch=master)](https://travis-ci.com/Ibujah/compactskel)

# One-step compact skeletonization

This software is an implementation of the compact skeletonization method.

One-step compact skeletonization, Durix B., Morin G., Chambon S., Mari J.-L. and Leonard K., Eurographics 2019

## Website:

http://durix.perso.enseeiht.fr/

## Needed libraries:

 * [Boost](http://www.boost.org/)
 * [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) (header only)
 * [OpenCV 3](http://opencv.org/)

## Instructions

Install the needed libraries

```
mkdir build
cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

The program is now in bin/

To use it on an example:

```
./soft_2dskeletonization --img ../ressources/rat.png --output
```

To compute a Voronoi skeleton:

```
./soft_2dskeletonization --img ../ressources/rat.png --epsilon 0.0 --output
```

## Misc

If you find a bug, or have a question, do not hesitate to ask!
