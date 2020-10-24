import yaml
import time
from prompt_toolkit import prompt, HTML, PromptSession
from prompt_toolkit.patch_stdout import patch_stdout
from prompt_toolkit.shortcuts import button_dialog, message_dialog
from prompt_toolkit.completion import NestedCompleter
from prompt_toolkit.history import FileHistory


class PromptHandler:
    def __init__(self, driver):
        cmd_history = FileHistory(".asd_command_history")
        self.points = yaml.load(open("./config/waypoints.yaml"), Loader=yaml.FullLoader)
        self.driver_instance = driver
        self.session = PromptSession(history=cmd_history)
        self.completer = NestedCompleter.from_nested_dict(
            {"goto": {"waypoint": None, "manual": None}, "exit": None, "pass": None}
        )

    def get_prompt_base(self):
        ozu_rover = ["<red>O</red>", "z", "<lightblue>U</lightblue>", " Rover"]
        current_loc = "Marsyard position is x=<b>{:.2f}</b> y=<b>{:.2f}</b> r=<b>{:.2f}</b>".format(
            *self.driver_instance.get_current_loc()
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
                elif len(segments) == 2 and segments[1] == "waypoint":
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
                    elif segments[1] == "waypoint":
                        try:
                            point = self.points[segments[1]][segments[2]]
                            x, y = point["x"], point["y"]
                        except Exception:
                            message_dialog(
                                title="Error", text="Entered option is invalid",
                            ).run()
                            continue
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
