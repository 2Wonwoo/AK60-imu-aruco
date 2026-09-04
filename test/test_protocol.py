from ak60_driver.protocol import Ak60V3Protocol


def test_confirmed_velocity_frames():
    protocol = Ak60V3Protocol()
    assert protocol.velocity_command(0.0).hex().upper() == "0003338000800800"
    assert protocol.velocity_command(2.0).hex().upper() == "0003338000892800"
    assert protocol.velocity_command(-2.0).hex().upper() == "000333800076E800"
    assert protocol.velocity_command(5.0).hex().upper() == "000333800096D800"
    assert protocol.velocity_command(-5.0).hex().upper() == "0003338000693800"


def test_control_ids():
    protocol = Ak60V3Protocol()
    assert [protocol.control_id(i) for i in range(1, 5)] == [
        0x801, 0x802, 0x803, 0x804
    ]

