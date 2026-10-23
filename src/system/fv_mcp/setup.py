from setuptools import setup

package_name = 'fv_mcp'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Takashi Otsuka',
    maintainer_email='takatronix@gmail.com',
    description='MCP server for LLM-driven FluentVision control',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fv_mcp_server = fv_mcp.server:main',
        ],
    },
)
