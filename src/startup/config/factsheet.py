from os import getenv

import robotics_api.generated.actions as actions
from localization_switcher.localization_switcher_node import LocalizationMethod
from order_management.factsheet_factory import FactsheetFactory


def get() -> FactsheetFactory:
    factory = FactsheetFactory()

    factory.set_manufacturer("ITL")
    robot_name = getenv("ROBOT_NAME")
    assert robot_name is not None
    factory.set_serial_number(robot_name)
    factory.set_name(robot_name)
    factory.set_dimensions(length=0.76, width=0.64, height=0.4)
    factory.set_kinematic("omni")

    factory.set_linear_velocity(0, 0)
    factory.set_angular_velocity(0, 0)
    factory.set_acceleration(0, 0)

    factory.set_localization_methods(
        # LocalizationMethod.AMCL,
        LocalizationMethod.INFRASTRUCTURE,
    )

    factory.add_actions(
        actions.reduceNodeOffset,
        actions.setLocalizationMethod,
        actions.updateLoads,
    )

    return factory
