import yaml
from prompt_toolkit import prompt, HTML, PromptSession
from prompt_toolkit.patch_stdout import patch_stdout
from prompt_toolkit.shortcuts import button_dialog, message_dialog
from prompt_toolkit.completion import NestedCompleter


class PromptHandler:
    def __init__(self, driver):
        self.points = yaml.load(open("./config/waypoints.yaml"), Loader=yaml.FullLoader)
        self.driver_instance = driver
        self.session = PromptSession()
        self.completer = NestedCompleter.from_nested_dict(
            {
                "goto": {
                    "waypoint": {k: None for k in self.points["waypoint"].keys()},
                    "scientific": {k: None for k in self.points["scientific"].keys()},
                    "manual": None,
                },
                "exit": None,
            }
        )

    def get_prompt_base(self):
        ozu_rover = ["<red>O</red>", "z", "<lightblue>U</lightblue>", " Rover"]
        error_num = self.driver_instance.M_T_err / 8
        if error_num <= 0.5:
            error = "<green>\u00B1{:.3f}</green>".format(error_num)
        elif error_num < 1.5:
            error = "<orange>\u00B1{:.3f}</orange>".format(error_num)
        else:
            error = "<red>\u00B1{:.3f}</red>".format(error_num)
        current_loc = "Marsyard position is x=<b>{:.2f}</b> y=<b>{:.2f}</b> <b>{}</b> meters".format(
            *self.driver_instance.get_current_loc(True), error
        )
        return HTML(
            "\n<b>{} - <i>ASD Driver</i></b>\n{}\n<b>$> </b>".format(
                "".join(ozu_rover), current_loc
            )
        )

    def query(self):
        with patch_stdout():
            while 1:
                result = self.session.prompt(
                    self.get_prompt_base(), completer=self.completer
                )
                segments = result.split()

                if len(segments) == 0:
                    continue

                if len(segments) == 1 and segments[0] == "exit":
                    confirm = button_dialog(
                        title="Exit Program",
                        text="Are you sure you want to exit?",
                        buttons=[("No", False), ("Yes", True)],
                    ).run()
                    if confirm:
                        raise Exception("User decided to exit")
                elif len(segments) == 1:
                    message_dialog(
                        title="Error", text="You have typed incomplete command",
                    ).run()
                    continue
                elif len(segments) == 2 and (
                    segments[1] == "waypoint" or segments[1] == "scientific"
                ):
                    message_dialog(
                        title="Error",
                        text="Please enter the name of the point you want to go",
                    ).run()
                    continue
                else:
                    if segments[1] == "manual":
                        if len(segments) == 4:
                            try:
                                x, y = float(segments[2]), float(segments[3])
                            except ValueError:
                                message_dialog(
                                    title="Error",
                                    text="Please enter float or integers",
                                ).run()
                                continue
                        else:
                            message_dialog(
                                title="Error",
                                text="Please enter coordinates as x and y",
                            ).run()
                            continue
                    elif segments[1] == "waypoint" or segments[1] == "scientific":
                        point = self.points[segments[1]][segments[2]]
                        x, y = point["x"], point["y"]
                    else:
                        message_dialog(
                            title="Error", text="Entered option is invalid",
                        ).run()
                        continue

                    confirm = button_dialog(
                        title="Go To New Point",
                        text="Are you sure you want to go to ({:.2f}, {:.2f})?".format(
                            x, y
                        ),
                        buttons=[("No", False), ("Yes", True)],
                    ).run()
                    if confirm:
                        return x, y
