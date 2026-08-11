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
    factory.set_dimensions(0.43, 0.3, 0.53)
    factory.set_kinematic("omni")

    factory.set_linear_velocity(0, 0)
    factory.set_angular_velocity(0, 0)
    factory.set_acceleration(0, 0)

    factory.set_localization_methods(
        # LocalizationMethod.AMCL,
        LocalizationMethod.INFRASTRUCTURE,
    )

    factory.add_actions(
        actions.detectApriltag,
        actions.finePositioning,
        actions.setControlCircuitNavigationMode,
        actions.setNav2NavigationMode,
        actions.reduceNodeOffset,
        # actions.setLocalizationMethod,
        actions.updateLoads,
    )

    return factory
