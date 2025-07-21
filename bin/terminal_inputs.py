import argparse


def terminal_inputs():
    """Parse the terminal inputs and return the arguments"""

    parser = argparse.ArgumentParser(
        prog="keelson_connector_anello",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument(
        "-l",
        "--log-level",
        type=int,
        default=30,
        help="Log level 10=DEBUG, 20=INFO, 30=WARN, 40=ERROR, 50=CRITICAL 0=NOTSET",
    )

    parser.add_argument(
        "--mode",
        "-m",
        dest="mode",
        choices=["peer", "client"],
        type=str,
        help="The zenoh session mode.",
    )

    parser.add_argument(
        "--connect",
        action="append",
        type=str,
        help="Endpoints to connect to, in case multicast is not working. ex. tcp/localhost:7447",
    )

    parser.add_argument(
        "-r",
        "--realm",
        default="rise",
        type=str,
        help="Unique id for a domain/realm to connect ex. rise",
    )

    parser.add_argument(
        "-e",
        "--entity-id",
        default="masslab",
        type=str,
        help="Entity being a unique id representing an entity within the realm",
    )

    parser.add_argument(
        "-s",
        "--source-id",
        default="ins/0",
        type=str,
        help="Source id being a unique id representing the source of the data",
    )

    parser.add_argument(
        "-f", 
        "--frame-id", 
        type=str,
        default=None, 
        required=False
    )

    parser.add_argument(
        "-p",
        "--publish",
        choices=["raw","imu", "ins","pos"],
        type=str,
        required=False,
        action="append",
    )

    parser.add_argument(
        "--udp-host",
        type=str,
        required=False,
        default="127.0.0.1",
        help="UDP host Anello data is sent to",
    )

    parser.add_argument(
        "--udp-port-data",
        type=int,
        required=False,
        default=2033,
        help="UDP port Anello data is sent to",
    )
    
    # Parse arguments and start doing our thing
    args = parser.parse_args()

    return args
