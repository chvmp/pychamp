from setuptools import setup

setup(
    name='pychamp',
    version='0.0.1',    
    description='CHAMP Native Python package',
    url='https://github.com/chvmp/pychamp',
    author='Juan Miguel Jimeno',
    author_email='jimenojmm@gmail.com',
    license='BSD 3-clause',
    packages=['champ',
              'champ.controllers',
              'champ.visualizer',
              'champ.robots',
              'champ.pybullet'],
    include_package_data=True,
    package_data = {'champ': ['robots/aliengo/*', 'robots/aliengo/meshes/*',
                              'robots/anymal_b/*', 'robots/anymal_b/meshes/*',
                              'robots/anymal_c/*', 'robots/anymal_c/meshes/*',
                              'robots/dkitty/*', 'robots/dkitty/meshes/*',
                              'robots/littledog/*', 'robots/littledog/meshes/*',
                              'robots/mini_cheetah/*', 'robots/mini_cheetah/meshes/*',
                              'robots/open_quadruped/*', 'robots/open_quadruped/meshes/*',
                              'robots/opendog/*', 'robots/opendog/meshes/*',
                              'robots/spot/*', 'robots/spot/meshes/*',
                              'robots/spotmicro/*', 'robots/spotmicro/meshes/*',
                              'robots/stochlite/*', 'robots/stochlite/meshes/*']
    },
    install_requires=['numpy',
                      'matplotlib',
                      'urdf_parser_py'],

    classifiers=[
        'Development Status :: 1 - Planning',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: BSD License',  
        'Operating System :: POSIX :: Linux',    
        'Operating System :: MacOS :: MacOS X',
        'Operating System :: Microsoft :: Windows',
        'Operating System :: POSIX',    
        'Programming Language :: Python :: 2',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.4',
        'Programming Language :: Python :: 3.5',
    ],
)