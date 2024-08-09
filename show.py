import re
from PIL import Image, ImageDraw, ImageFont
import math

class SchematicElement:
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)

    def draw(self, draw):
        pass

class Component(SchematicElement):
    def __init__(self, x, y, reference, value):
        super().__init__(x, y)
        self.reference = reference
        self.value = value

    def draw(self, draw):
        draw.rectangle([self.x-20, self.y-10, self.x+20, self.y+10], outline="black")
        font = ImageFont.load_default()
        draw.text((self.x-15, self.y-5), self.reference, fill="black", font=font)

class Wire(SchematicElement):
    def __init__(self, x1, y1, x2, y2):
        super().__init__(x1, y1)
        self.x2 = float(x2)
        self.y2 = float(y2)

    def draw(self, draw):
        draw.line([self.x, self.y, self.x2, self.y2], fill="black")

def parse_schematic(filename):
    elements = []
    with open(filename, 'r') as file:
        content = file.read()

    # Parse components
    component_pattern = r'\(symbol \(lib_id "[^"]+"\) \(at (\d+\.?\d*) (\d+\.?\d*)[^)]+\)\s+\(property "Reference" "([^"]+)"[^)]+\)\s+\(property "Value" "([^"]+)"'
    for match in re.finditer(component_pattern, content):
        x, y, reference, value = match.groups()
        elements.append(Component(x, y, reference, value))

    # Parse wires
    wire_pattern = r'\(wire \(pts \(xy (\d+\.?\d*) (\d+\.?\d*)\) \(xy (\d+\.?\d*) (\d+\.?\d*)\)\)'
    for match in re.finditer(wire_pattern, content):
        x1, y1, x2, y2 = match.groups()
        elements.append(Wire(x1, y1, x2, y2))

    return elements

def render_schematic(elements, filename):
    # Find the bounds of the schematic
    min_x = min(min(e.x for e in elements), min(getattr(e, 'x2', float('inf')) for e in elements))
    max_x = max(max(e.x for e in elements), max(getattr(e, 'x2', float('-inf')) for e in elements))
    min_y = min(min(e.y for e in elements), min(getattr(e, 'y2', float('inf')) for e in elements))
    max_y = max(max(e.y for e in elements), max(getattr(e, 'y2', float('-inf')) for e in elements))

    # Add some padding
    padding = 50
    width = int(max_x - min_x + 2*padding)
    height = int(max_y - min_y + 2*padding)

    img = Image.new('RGB', (width, height), color='white')
    draw = ImageDraw.Draw(img)

    for element in elements:
        # Adjust coordinates
        element.x -= min_x - padding
        element.y -= min_y - padding
        if isinstance(element, Wire):
            element.x2 -= min_x - padding
            element.y2 -= min_y - padding
        element.draw(draw)

    img.save(filename)

# Usage
elements = parse_schematic("imu.kicad_sch")
render_schematic(elements, "schematic.png")