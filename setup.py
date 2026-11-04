from setuptools import setup, find_packages

if __name__ == "__main__":
    setup(include_package_data=True,
    packages=find_packages(),
    package_data={"": ["VERSION", "*.res"]},
    data_files=[("RCAIDE", ["RCAIDE/VERSION"])],)