cmake_minimum_required(VERSION 2.6)
project(Rabbits_Nucleo_Platform)

find_package(Rabbits REQUIRED)


add_subdirectory(components)
# add_subdirectory(plugins)

rabbits_add_configs(nucleo-f401re.yml)
#rabbits_add_configs(lucneo.yml)

rabbits_add_dynlib(nucleo)
