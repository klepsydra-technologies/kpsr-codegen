# Klepsydra SDK Code Generation Tool

## System dependencies

* Python 3.6 or above

build module is needed:

```
        python3 -m pip install build
```

## Installation instructions 

Clone the repo and submodules: 

```
        git submodule update --init --recursive 
```

and use the following commands to build and install the code generation tool:


```
        python -m unittest
        python3 -m build --outdir dist .
        pip3 install dist/kpsr_codegen-1.0-py3-none-any.whl
```

Run the cpp tests:

```
        cd cpp_testing
        mkdir build
        cd build
        cmake -DCMAKE_PREFIX_PATH=/home/kpsruser/development/klepsydra/professional/install ..
        make
        make test
```


## Usage

See the [Tutorial](https://github.com/klepsydra-technologies/kpsr-tutorial/blob/main/tutorials/chapter3.md) for detailed usage of the tool.

The options that can be provided to
A simple help is available by passing the `-h` option:

```
$kpsr_codegen -h
kpsr_codegen [options]: 
   Input directory: -i | --idir
   Output directory: -o | --odir
   Include path: -p | --include_path
   Disable DDS: -d | --disable_dds
   Disable ZMQ: -z | --disable_zmq
   Disable ROS: -r | --disable_ros
   Configuration path: -c | --conf_path
   Template path: -t | --template_path
```

By default, code generation is *enabled* for all the middlewares. Pass `True` to disable code generation for specific middlewares.
