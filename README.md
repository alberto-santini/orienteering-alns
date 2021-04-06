# Installation

First, download the source code:

```
git clone https://github.com/alberto-santini/orienteering-alns.git
cd orienteering-alns
```

### Prerequisites

The programme can be built in a straightforward way, provided that all required libraries are available.

```{sh}
mkdir third_party
```

These are:

##### 1. The CPLEX solver by IBM, including its C++ API and the Concert API.

```{sh}
export CPLUS_INCLUDE_PATH=<CPLEX_STUDIO_DIR>/cplex/include/:<CPLEX_STUDIO_DIR>/concert/include/
export LIBRARY_PATH=<CPLEX_STUDIO_DIR>/cplex/lib/x86-64_linux/static_pic/:<CPLEX_STUDIO_DIR>/concert/lib/x86-64_linux/static_pic/
```

> You might need to change `x86-64_linux` with the appropriate directory name.

##### 2. The [LKH](http://akira.ruc.dk/~keld/research/LKH/) heuristic solver for the Travelling Salesman Problem.

```{sh}
wget http://akira.ruc.dk/~keld/research/LKH/LKH-2.0.9.tgz
tar xvzf LKH-2.0.9.tgz
cd  LKH-2.0.9
make
mkdir -p ~/local/bin/
cp LKH ~/local/bin/
cd ..
```

##### 3. The [CImg](http://cimg.eu/) image manipulation library.

```{sh}
git clone --depth=1 https://github.com/dtschump/CImg.git
cp CImg/CImg.h third_party/
```

##### 4. The [ProgramOptions.hxx](https://github.com/Fytch/ProgramOptions.hxx) library.

```{sh}
wget https://raw.githubusercontent.com/Fytch/ProgramOptions.hxx/master/include/ProgramOptions.hxx -P thrird_party
```

##### 5. My own library of miscellaneous utilities, called AS and [available on GitHub](https://github.com/alberto-santini/as).

```{sh}
git clone https://github.com/alberto-santini/as.git
mkdir third_party/as
cp as/src/* third_party/as/
```

##### 6. My ALNS (Adaptive Large Neighbourhood Search) library, also [available on GitHub](https://github.com/alberto-santini/adaptive-large-neighbourhood-search).

```{sh}
git clone https://github.com/alberto-santini/adaptive-large-neighbourhood-search.git
mkdir third_party/palns
cp -r adaptive-large-neighbourhood-search/src/* third_party/palns/
```

### Build

Once the required libraries are installed, build it with:

```{sh}
cd source-code
g++ -o orienteering-alns -O3 -std=c++17                          \
  -I../third_party                                               \
  *.cpp palns/*.cpp                                              \
  -lm -lpthread -lX11  -lstdc++fs  -lconcert -lilocplex -lcplex
```


# License

The programme is distributed under the GNU General Public License, version 3.
See the `LICENSE` file.
