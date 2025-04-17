from setuptools import setup, Extension
import sys
import setuptools
from setuptools.command.build_ext import build_ext
import os

class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)

class CMakeBuild(build_ext):
    def run(self):
        try:
            import cmake
        except ImportError:
            print("CMake must be installed to build the extension")
            raise

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        
        cmake_args = ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
                     '-DPYTHON_EXECUTABLE=' + sys.executable]

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
        build_args += ['--', '-j4']

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(
            env.get('CXXFLAGS', ''),
            self.distribution.get_version())
        
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
            
        import subprocess
        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, 
                            cwd=self.build_temp, env=env)
        subprocess.check_call(['cmake', '--build', '.'] + build_args,
                            cwd=self.build_temp)

setup(
    name='pydecomp_util',
    version='0.1',
    author='Original authors',
    description='Python bindings for DecompUtil library',
    long_description='',
    ext_modules=[CMakeExtension('pydecomp_util')],
    cmdclass=dict(build_ext=CMakeBuild),
    zip_safe=False,
    python_requires='>=3.6',
    install_requires=[
        'numpy>=1.19.0',
        'pybind11>=2.6.0',
        'cmake>=3.18.0',
    ],
    setup_requires=[
        'pybind11>=2.6.0',
        'cmake>=3.18.0',
    ]
)