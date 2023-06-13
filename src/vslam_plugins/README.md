# Create a new VSLAM plugin

I try to follow the [dual-hierarchy](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#c129-when-designing-a-class-hierarchy-distinguish-between-implementation-inheritance-and-interface-inheritance) structure, where the implementation and interface inheritance are separated.
Creating a new interface inheritance indicates we need new interfaces, and we implement the interfaces in the implementation inheritance.
We define the interfaces in abstract classes wrapped in the `base` or `abstract` namespace. 