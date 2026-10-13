import os
import random
import unittest
import xml.etree.ElementTree as ET

from house_placement import BUILDING_NAMES, generate_centers, place_buildings


BASE_WORLD = os.path.join(os.path.dirname(__file__), '..', '..',
                          'waypoint', 'world', 'base1.world')


class BuildingPlacementTests(unittest.TestCase):
    def test_each_building_receives_its_own_pose(self):
        with open(BASE_WORLD, encoding='utf-8') as source:
            base_text = source.read()
        centers = [(10 * index, -10 * index) for index in range(len(BUILDING_NAMES))]
        world = ET.fromstring(place_buildings(base_text, centers)).find('world')
        models = {model.get('name'): model for model in world.findall('model')}

        self.assertFalse(world.findall('state'))
        for name, expected in zip(BUILDING_NAMES, centers):
            pose = models[name].find('pose')
            if pose is None:
                pose = models[name].find('link/pose')
            self.assertEqual(tuple(map(float, pose.text.split()[:2])), expected, name)
        self.assertIsNone(models['gas_station_73'].find('pose'))

    def test_failed_sampling_never_returns_overlapping_centers(self):
        random.seed(1234)
        with self.assertRaises(RuntimeError):
            generate_centers(1, lambda _: -30, attempts=1)


if __name__ == '__main__':
    unittest.main()
