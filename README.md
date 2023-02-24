![Tourist trip planning](tourist-trip.jpg)
<sub><sub>Tourist trip planning is a common application of the Orienteering Problem! Photo released on pxhere.com under License CC0 1.0 Universal (CC0 1.0) Public Domain Dedication</sub></sub>

# ALNS for the Orienteering Problem

This repository contains a heuristic algorithm for the Orienteering Problem based on the Adaptive Large Neighbourhood Search paradigm.
Folder `source-code` contains the source code of the solver.
Folder `complete-results` contains the results used in the below paper.

## Citation

If you use this software, please cite the following paper:

```bib
@article{santini2019op,
  title={An {Adaptive Large Neighbourhood Search} algorithm for the {Orienteering Problem}},
  author={Santini, Alberto},
  journal={{Expert Systems with Applications}},
  volume=123,
  year=2019,
  pages={154--167},
  doi={10.1016/j.eswa.2018.12.050}
}
```

You can also cite this repository through Zenodo.

[![DOI](https://zenodo.org/badge/131393728.svg)](https://zenodo.org/badge/latestdoi/131393728)

```bib
@misc{alns_op_github,
    title={Adaptive Large Neighbourhood Search for the Orienteering Problem},
    author={Santini, Alberto},
    date={2018-10-27},
    howpublished={Github repository},
    doi={10.5281/zenodo.1472850},
    url={https://github.com/alberto-santini/orienteering-alns/}
}
```

## Prerequisites

The programme can be built in a straightforward way, provided that all required libraries are available.
These are:

* The CPLEX solver by IBM, including its C++ API and the Concert API.
* The [LKH](http://akira.ruc.dk/~keld/research/LKH/) heuristic solver for the Travelling Salesman Problem.
* The [CImg](http://cimg.eu/) image manipulation library.
* The [ProgramOptions.hxx](https://github.com/Fytch/ProgramOptions.hxx) library.
* My own library of miscellaneous utilities, called AS and [available on GitHub](https://github.com/alberto-santini/as).
* My ALNS (Adaptive Large Neighbourhood Search) library, also [available on GitHub](https://github.com/alberto-santini/adaptive-large-neighbourhood-search).

## License

The programme is distributed under the GNU General Public License, version 3.
See the `LICENSE` file.
