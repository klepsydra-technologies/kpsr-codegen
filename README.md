# Klepsydra SDK Code Generation Tool

## System dependencies

* Python 3.6 or above

## Installation instructions 

Clone the repo and use the following commands to build and install the code generation tool:

```
        python3 -m build
        pip3 install dist/kpsr_codegen-1.0-py3-none-any.whl
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
