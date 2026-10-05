from pathlib import Path
from setuptools import setup

data = [
    ('share/ament_index/resource_index/packages', ['resource/mfr3duo_scenes']),
    ('share/mfr3duo_scenes', ['package.xml', 'README.md']),
]
for directory in sorted({p.parent for p in Path('scenes').rglob('*')
                         if p.is_file() and not any(part.startswith('.') for part in p.parts)}):
    data.append(('share/mfr3duo_scenes/' + str(directory),
                 [str(p) for p in sorted(directory.iterdir()) if p.is_file()]))
setup(name='mfr3duo_scenes', version='0.1.0', packages=['mfr3duo_scenes'],
      data_files=data, install_requires=['setuptools', 'PyYAML'],
      description='Portable kitchen and robot scene composition',
      license='Apache-2.0', zip_safe=False)
