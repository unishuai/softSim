# ============================================
# @File    : setup.py
# @Date    : 2024-07-29 14:51
# @Author  : 帅宇昕
# ============================================
from setuptools import setup, find_packages
setup(
    name='softSim',  # 包名称
    version='0.1.0',  # 包版本
    author='unishuai',  # 作者名称
    author_email='unishuai@gmail.com',  # 作者邮箱
    description='软件建模仿真项目',  # 包的简短描述
    long_description=open('README.md').read(),  # 包的详细描述，通常从 README 文件读取
    long_description_content_type='text/markdown',  # 详细描述的格式
    url='https://github.com/unishuai',  # 项目的主页
    packages=find_packages(),  # 自动查找并包含所有包
    classifiers=[
        'Programming Language :: Python :: 3',  # 指定Python版本
        'License :: OSI Approved :: MIT License',  # 许可证信息
    ],
    python_requires='>=3.9',  # 指定支持的Python版本
    install_requires=[
        'requests',
    ],
    include_package_data=True,  # 是否包含包内的非Python文件

)