from pydantic import PlainSerializer

EnumNameSerializer = PlainSerializer(
    lambda e: e.name.lower(), return_type=str, when_used="always"
)
