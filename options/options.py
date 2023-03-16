import argparse
from dataclasses import dataclass

import yaml


@dataclass
class Options():
    """"Dataclass for housing experiment flags."""

    random_seed: int = 0

    # number of views the model should expect in a tuple.
    model_num_views: int = 8

    # image size input to the network. Used in dataloaders and projectors.
    image_width: int = 512
    image_height: int = 384

    ################################## models ##################################
    # loads model weights
    load_weights_from_checkpoint: str = None

    # image prior encoder
    image_encoder_name: str = "efficientnet"

    # final depth decoder.
    depth_decoder_name: str = "unet_pp"

    # loss
    loss_type: str = "log_l1"

    # matching encoder. resnet or fpn
    matching_encoder_type: str = "resnet"
    # number of channels for matching features
    matching_feature_dims: int = 16

    # scale to match features at. 1 means half the final depth output size, or a
    # quarter of image resolution.
    matching_scale: int = 1
    # number of depth bins to use in the cost volume.
    matching_num_depth_bins: int = 64
    # min and max depth planes in the cost volume
    min_matching_depth: float = 0.25
    max_matching_depth: float = 5.0

    # type of cost volume encoder.
    cv_encoder_type: str = "multi_scale_encoder"

    # type of cost volume to use. SimpleRecon's metadata model uses the 
    # 'mlp_feature_volume' model. Also available in this repo is a simple dot
    # reduction model 'simple_cost_volume'
    feature_volume_type: str = "mlp_feature_volume"


class OptionsHandler():
    """A class for handling experiment options.

        This class handles options files and optional CLI arguments for
        experimentation.
        
        The intended use looks like this:

            optionsHandler = options.OptionsHandler()
            # uses a config filename from args or populates flags from CLI
            optionsHandler.parse_and_merge_options()

            # optionally print
            optionsHandler.pretty_print_options()

        You could also load from a config file you choose and ignore one that 
        may be supplied in args.
            optionsHandler.parse_and_merge_options(config_filepath =
                                    os.path.join("configs", "test_config.yaml"))

        Options will be populated by an optional supplied config files first, 
        then overwritten by any changes provided in command line args. If a 
        required attribute is not defined in either, then an Exception is thrown.
        I want to add a new arg! What should I do? Well, easy. Add an entry in 
        the Options class and specify a type and default value. If this needs to 
        be a required arg, set None for a default value and also add its name
        as a string to the required_flags list in the OptionsHandler class's 
        initializer.

        There are two config files allowed. --config_file, then 
        --data_config_file. Order of overriding (last overrides above): 
            - config_file
            - data_config_file
            - CLI arguments

    """
    def __init__(self, required_flags=[]):
        """Sets up the class and stores required flags."""
        if required_flags is None:
            required_flags = []

        self.options = Options()
        self.required_flags = required_flags

        self.parser = argparse.ArgumentParser(description="SimpleRecon Options")
        self.parser.add_argument('--config_file', type=str, default=None)
        self.parser.add_argument('--data_config_file', type=str, default=None)


        self.populate_argparse()

    def parse_and_merge_options(self, config_filepaths=None, ignore_cl_args=False):
        """Parses flags from a config file and CL arguments.
            Args:
                config_filepaths: str filepath to a .yaml or list of filepaths 
                to config files ignore_cl_args: optionally ignore CLI 
                altogether, useful for debugging with a hardcoded config 
                filepath and in python notebooks.

            Raises:
                Exception: raised when required arguments aren't satisfied.
        """
        # parse args
        if not ignore_cl_args:
            cl_args = self.parser.parse_args()

        # load config file
        if config_filepaths is not None:
            # use config_filepath(s) provided here if available
            if isinstance(config_filepaths, list):
                for config_filepath in config_filepaths:
                    config_options = OptionsHandler.load_options_from_yaml(
                                                                config_filepath)
                    self.merge_config_options(config_options)
            else:
                config_options = OptionsHandler.load_options_from_yaml(
                                                               config_filepaths)
                self.merge_config_options(config_options)

            self.config_filepaths = config_filepaths

        elif (not ignore_cl_args and 
            (cl_args.config_file is not None or 
                cl_args.data_config_file is not None)):
            # if args tells us we should load from a file, then let's do that.
            self.config_filepaths = []

            # add from standard config first
            if cl_args.config_file is not None:
                config_options = OptionsHandler.load_options_from_yaml(
                                                            cl_args.config_file)
                self.merge_config_options(config_options)
                self.config_filepaths.append(cl_args.config_file)

            # then merge from a data config
            if cl_args.data_config_file is not None:
                config_options = OptionsHandler.load_options_from_yaml(
                                                    cl_args.data_config_file)
                self.merge_config_options(config_options)
                self.config_filepaths.append(cl_args.data_config_file)
        else:
            # no config has been supplied. Let's hope that we have required
            # arguments through command line.
            print("Not reading from a config_file.")
            config_options = None
            self.config_filepaths = None
            

        if not ignore_cl_args:
            # merge args second and overwrite everything that's come before
            self.merge_cl_args(cl_args)

        # now check that all required arguments are satisfied
        self.check_required_items()

    def populate_argparse(self):
        """Populates argparse arguments using Options attributes."""

        for field_name in self.options.__dataclass_fields__.keys():
            field_info = self.options.__dataclass_fields__[field_name]
            if field_info.type == bool:
                self.parser.add_argument(f'--{field_name}', action="store_true")
            else:
                self.parser.add_argument(
                                        f'--{field_name}', 
                                        type=field_info.type, 
                                        default = None,
                                    )

    def check_required_items(self):
        """Raises a flag if options isn't defined."""
        for required_flag in self.required_flags:
            if self.options.__getattribute__(required_flag) is None:
                raise Exception(f"Error! Missing required config argument '{required_flag}'")

    def merge_config_options(self, config_options):
        """"""

        # loop over loaded config and update those in self.options.
        for field_name in config_options.__dict__.keys():
            value = config_options.__getattribute__(field_name)
            self.options.__setattr__(field_name, value)

    def merge_cl_args(self, cl_args):
        # loop over loaded args and update those in self.options.
        for arg_pair in cl_args._get_kwargs():
            # this should be the only argument that doesn't match here.
            if arg_pair[0] == "config_file":
                continue

            if arg_pair[1] is not None:
                # check if type bool and if false, in that case ignore
                if isinstance(arg_pair[1], bool) and not arg_pair[1]:
                    continue

                if arg_pair[0] == "prediction_mlp_channels":
                    array = "".join(arg_pair[1]).split("_")
                    array = [int(dim) for dim in array]
                    self.options.__setattr__(arg_pair[0], array)
                else:
                    self.options.__setattr__(arg_pair[0], arg_pair[1])


    def pretty_print_options(self):
        print("########################### Options ###########################")
        print("")
        for field_name in self.options.__dataclass_fields__.keys():
            print("    ", field_name + ":", self.options.__getattribute__(field_name))
        print("")
        print("###############################################################")

    @staticmethod
    def load_options_from_yaml(config_filepath):
        stream = open(config_filepath, 'r')
        return yaml.load(stream, Loader=yaml.Loader)

    @staticmethod
    def save_options_as_yaml(config_filepath, options):
        with open(config_filepath, 'w') as outfile:
            yaml.dump(options, outfile, default_flow_style=False)

def handle_backwards_compat(opts):
    # modify older experiment configs if needed
    return opts
