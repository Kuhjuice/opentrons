from pytest import raises

from pydantic import BaseModel, ValidationError
from system_server.service.resource_links import ResourceLinks


class ThingWithLink(BaseModel):
    links: ResourceLinks


def test_follows_structure() -> None:
    structure_to_validate = {
        "links": {
            "self": {"href": "/items/1", "meta": None},
        }
    }
    validated = ThingWithLink.parse_obj(structure_to_validate)
    assert validated.dict() == structure_to_validate


def test_must_be_self_key_with_string_value() -> None:
    invalid_structure_to_validate = {
        "invalid": {
            "key": "value",
        }
    }
    with raises(ValidationError) as e:
        ThingWithLink.parse_obj(invalid_structure_to_validate)
    assert e.value.errors() == [
        {"loc": ("links",), "msg": "field required", "type": "value_error.missing"}
    ]
