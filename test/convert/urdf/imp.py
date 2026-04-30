import unittest, io
import robmodel.connectivity
import robmodel.ordering

import robmodel.convert.urdf.imp as importer

class URDFImportTests(unittest.TestCase):
    def test_basic(self):
        model = '''
<robot name="Foo">
    <link name="base"></link>
    <link name="shoulder"></link>
    <joint name="shoulder_pan" type="revolute">
        <parent link="base"/>
        <child  link="shoulder"/>
    </joint>
</robot>'''

        urdf = importer.URDFWrapper(io.StringIO(model))
        connectivity, ordering = importer.convert(urdf)[0:2]
        self.assertEqual(connectivity.nB, 2)
        self.assertEqual(connectivity.nJ, 1)


    def test_drop_fixed(self):
        model = '''
<robot name="Foo">
    <link name="base"></link>
    <link name="shoulder"></link>
    <link name="payload"></link>
    <joint name="shoulder_pan" type="revolute">
        <parent link="base"/>
        <child  link="shoulder"/>
    </joint>
    <joint name="rigid_attachment" type="fixed">
        <parent link="shoulder"/>
        <child  link="payload"/>
    </joint>
</robot>'''

        urdf = importer.URDFWrapper(io.StringIO(model))
        connectivity, ordering = importer.convert(urdf)[0:2]
        self.assertEqual(connectivity.nB, 3)
        self.assertEqual(connectivity.nJ, 2)

        connectivity, ordering = importer.convert(urdf, ignoreFixedJoints=True)[0:2]
        self.assertEqual(connectivity.nB, 2)
        self.assertEqual(connectivity.nJ, 1)

if __name__ == '__main__':
    unittest.main()
