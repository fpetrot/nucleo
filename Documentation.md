# Rabbits Documentation 

Rabbits is used to create board simulation using Qemu and SystemC together. 

## Platform 

### Folder organisation 

```
.
+-- platform.yml
+-- Components
    +-- Component1
        +-- Component1.cc
        +-- Component1.h
        +-- Component1.yml
    +-- Component2
        +-- ..
+-- Plugins
    +-- Plugin1
        +--...
```
Each folder has a CMakeLists.txt. The main rabbits commands are : 
- For platform folder : 
```
rabbits_add_configs(nucleo-f401re.yml) => add new platform
rabbits_add_dynlib(nucleo) => add platform library to rabbits

```

- For a component or plugin folder:
```
rabbits_add_sources(component.cc)
rabbits_add_components(component.yml)
rabbits_add_plugins(plugin.yml)
```

###  Platform Yaml 

All informations for a platform are stored in `platform.yml`. 

```Yaml
platforms:
  name-of-the-platform:
    description: A brief description 
    
    components: # List all the components inside the platform
    
    # Bus used between cpu and components
    mem-bus: 
      type: simple-bus
    
    # Bus Master  
    cpu: 
      type: cpu-type
      bindings: 
        mem:mem-bus
        ...
        
    # Generic Memory
    memory:
      type: memory
      size: &mem_size 512K # Parameter defined in memory.yml
      bindings: 
        mem:
          peer: mem-bus
          address: { &mem_start : 0x... : *mem_size }
          
    # A component example ( bus Slave ) 
    component1:
      type: type-of-the-component1
      bindings: 
        mem:
          peer: mem-bus
          address: { 0x... :  ... } 
        
    # A component which is connected to component1 
    component2:
      type: type-of-the-component2
      bindings:
        mem: 
          ...
        p-to-component1: # A port define inside component2
          peer: component1.p-from-component2 # p-from-component2 is defined inside component1 
          
    
```

If a port is inside a VectorPort, it can be accessed by adding its number at the end. For example, `peer: gpio-a.gpios5` will link to the fifth port in the VectorPort "gpios" inside component "gpio-a".


### Components creation 

Each component needs at least 3 files : 
- component.yml which will contains all informations for this component.
- component.h and component.cc which contain component implementation

####  component.yml
This file gives to the platform all information for this component ( type, which class used when called, etc ) and defines parameters.

```Yaml
component:
  implementation: component
  type: component-type
  class: ComponentClass
  include: component.h 
  description: ...
  parameters:
    param1:
      type: uint8 
      default: 0x10 #default value for this parameter 
      description: ...
```

####  component.h and component.cc

Those two files defined all the implementation of the component. Some examples : 

``` C++
Class ComponentClass : public Slave<> { 
  public: 
    // Mandatory
    SC_HAS_PROCESS(ComponentClass);
    Component(sc_core::sc_module_name name, const Parameters &params, ConfigManager &c);
    virtual ~ComponentClass();
    
    /* List of all ports. 
     * NOTE : Some are already defined in Slave<> like "mem".
     */
     OutPort<bool> p_irq;
     VectorPort < InOutPorts<bool> > p_gpios
      ...
      
  private:
    /* Read and write functions.
     * NOTE : Must override functions defined in port.h
     */
      void bus_cb_read(uint64_t addr, uint8_t *data, unsigned int len, bool &bErr); // read for all len
      void bus_cb_read(uint64_t addr, uint8_t *data, unsigned int len, bool &bErr); // write for all len
      void bus_cb_write_16(uint64_t addr, uint16_t *data, bool &bErr); // Specific action for 16 bits write

     // All internals variables like registers, sc_event
     ...
}
```

Component constructer is used to instantiate component but also to instantiate ports. The name given for the ports will be the one to use in `platform.yml`
``` C++
  ComponentClass::ComponentClass(sc_core::sc_module_name name, const Parameters &params, ConfigManager &c)
    : Slave(name, params, c)
    , p_gpios("gpios", NUCLEO_GPIO_COUNT) // Init of a port vector
    , p_irq("irq") // Init of a single port
  {
    m_param1 = params["params1"].as<uint8_t>(); // retrieve param1 value from Yaml
    
    // SystemC informations 
    SC_THREAD(thread1); 
    SC_METHODS(method1);
  }
```

### Logs / Debug

Rabbits provides logs and debugging informations which can be enable globally or component by component. To enable it globally, please refers to `./rabbits.sh -help`. To enable for only one component, you must change `log-level` parameters under component informations in the file `platform.yml` 
To add logs on a component implementation, use: 
```
MLOG_F(SIM, lvl, "Debugging informations :  val = 0x%x\n", val);
// lvl can be TRC?, DBG, INF ? , ERR.  
```

