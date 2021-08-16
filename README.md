# Multi-GNSS

Low cost Multi-GNSS system that has 3 [UbloxNeo-M8N](https://www.u-blox.com/en/product/neo-m8-series) modules in one unmanned ground vehicle. Data redundancy of the Multi-GNSS system allows a greater number of samples and better filtering data. Through experiments on different circuits, we obtained sample rates of 3 Hz.

<p float="center">
  <img src="/images/arduino_NeoM8.png" width="480"  />
  <img src="/images/blue_multiGNSS.png" width="325" /> 
</p>

## Hardware and Software

This system is implemented on the [Arduino Mega](https://raw.githubusercontent.com/EPVelasco/Multi-GNSS/main/images/arduino_NeoM8.png) platform and 3 [UbloxNeo-M8N](https://www.u-blox.com/en/product/neo-m8-series) modules. The location of the UGV is obtained by removing the measurements that have an out-of-range deviation. This data is published to ROS using the [rosserial](http://wiki.ros.org/rosserial) package.

## Experiments 
The results are exposed in the publication: "[Implementación y Evaluación de un sistema Multi-Gnss en un vehículo terrestre no tripulado](https://doi.org/10.17979/spudc.9788497498043.588)" [Pending publication]

## Results

<p float="center">
  <img src="/images/neoM8n-MultiGnss.png" width="460"  />
  <img src="/images/circuit_760meters.png" width="365" /> 
</p>

