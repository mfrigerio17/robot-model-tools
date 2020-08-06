import setuptools
import os, glob, shutil

# Get the long description from the README file
with open('readme.md', encoding='utf-8') as f:
    long_description = f.read()

class RealClean(setuptools.Command):
    """Custom clean command to tidy up the project root."""
    CLEAN_FILES = './build ./dist ./*.egg-info */*.egg-info'.split(' ')

    user_options = []

    def initialize_options(self):
        pass
    def finalize_options(self):
        pass

    def run(self):
        for path_spec in RealClean.CLEAN_FILES:
            for path in glob.glob(path_spec):#[str(p) for p in abs_paths]:
                print('removing {0}'.format(path))
                shutil.rmtree(path)

pkgs = setuptools.find_packages(where='src')
pkgs.extend( setuptools.find_namespace_packages(where='src', include=['robmodel.*']) )

setuptools.setup(
    name='robot-model-tools',
    version='0.1.0',
    description='Classes to model articulated robots, and a command line tool',
    long_description=long_description,
    long_description_content_type='text/markdown',
    author='Marco Frigerio',
    classifiers=[
        "Programming Language :: Python :: 3",
        'License :: OSI Approved :: BSD License',
        "Operating System :: OS Independent",
    ],

    # https://docs.python.org/3/distutils/setupscript.html#listing-whole-packages
    # include all packages under src/
    packages = pkgs,
    # tell setuptools packages are under src/
    package_dir = {'': 'src'},

    package_data = {
        # include the textX grammar
        "robmodel.convert.kindsl": ["*.tx"],
    },

    python_requires='>=3.3',

    install_requires = [
        'networkx',
        'numpy',
        'sympy',
        'pyyaml',
        'textX'
    ],

    entry_points = {
        "console_scripts": ["rmt = rmt.rmt:main" ]
    },

    cmdclass={
        'realclean': RealClean,
    }
)
