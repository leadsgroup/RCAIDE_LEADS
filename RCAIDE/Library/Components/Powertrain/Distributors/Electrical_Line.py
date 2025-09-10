
class Electrical_Line(Component):
    def __defaults__(self):
        self.tag = 'electrical_line'
        self.to = None
        self.from_ = None
        self.current_type = 'DC'  
        self.voltage = 0  
        self.efficiency = 1
        self.length = 0  
        self.diameter_conductor = 0.005  # Default conductor diameter
        self.diameter_insulator = 0.01  # Default insulator diameter
        self.conductor_material = Copper()  # Default conductor material
        self.insulator_material = Polyimide()  # Default insulator material
    
   