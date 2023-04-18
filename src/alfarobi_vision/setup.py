from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages = ['object_detection'],
    package_dir = {'' : 'include/yolov5_detector'}
)

setup(**d)
