from setuptools import setup, find_packages

setup(
    name="vlm_geolocator",
    version="2.0.0",
    description="Vision Language Model Geolocation System for drone-based target detection",
    author="LifGorg",
    author_email="",
    url="https://github.com/LifGorg/VLM_Geolocator",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    python_requires=">=3.8",
    install_requires=[
        "numpy",
        "opencv-python",
        "scipy",
        "PyYAML",
        "torch",
        "transformers",
        "Pillow",
    ],
    extras_require={
        "dev": [
            "pytest",
            "black",
            "flake8",
        ],
    },
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
    ],
)

