/*
 * Thunderbolt PCI Express Root Port
 *
 * This is the regular generic PCIe Root Port with ACPI device properties that
 * make Apple IOPCIFamily classify the downstream hierarchy as Thunderbolt.
 *
 * Copyright (C) 2026
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/acpi/aml-build.h"
#include "hw/acpi/pci.h"
#include "hw/acpi/acpi_aml_interface.h"
#include "qom/object.h"

#define TYPE_THUNDERBOLT_ROOT_PORT "thunderbolt-root-port"

static Aml *build_device_properties_dsd(Aml *properties)
{
    Aml *dsd;

    dsd = aml_package(2);
    aml_append(dsd, aml_touuid("DAFFD814-6EBA-4D8C-8A91-BC9BBF4AA301"));
    aml_append(dsd, properties);

    return aml_name_decl("_DSD", dsd);
}

static void append_bool_property(Aml *properties, const char *name)
{
    Aml *property = aml_package(2);

    aml_append(property, aml_string("%s", name));
    aml_append(property, aml_int(1));
    aml_append(properties, property);
}

static Aml *build_apple_device_properties_dsm(Aml *properties)
{
    Aml *method;
    Aml *ifctx;
    uint8_t supported_funcs[1] = { 0x03 };

    method = aml_method("_DSM", 4, AML_SERIALIZED);

    ifctx = aml_if(aml_equal(aml_arg(2), aml_int(0)));
    aml_append(ifctx, aml_return(aml_buffer(sizeof(supported_funcs),
                                            supported_funcs)));
    aml_append(method, ifctx);
    aml_append(method, aml_return(properties));

    return method;
}

static void append_apple_bool_property(Aml *properties, const char *name)
{
    aml_append(properties, aml_string("%s", name));
    aml_append(properties, aml_int(1));
}

static void build_thunderbolt_root_port_dsm(Aml *scope)
{
    Aml *properties;

    properties = aml_package(4);
    append_apple_bool_property(properties, "PCI-Thunderbolt");
    append_apple_bool_property(properties, "pci-supports-link-change");

    aml_append(scope, build_apple_device_properties_dsm(properties));
}

static void build_thunderbolt_root_port_dsd(Aml *scope)
{
    Aml *properties;

    properties = aml_package(2);
    append_bool_property(properties, "PCI-Thunderbolt");
    append_bool_property(properties, "pci-supports-link-change");

    aml_append(scope, build_device_properties_dsd(properties));
}

static void build_thunderbolt_root_port_aml(AcpiDevAmlIf *adev, Aml *scope)
{
    build_thunderbolt_root_port_dsd(scope);
    build_thunderbolt_root_port_dsm(scope);
    build_pci_bridge_aml(adev, scope);
}

static void thunderbolt_root_port_class_init(ObjectClass *klass,
                                             const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    AcpiDevAmlIfClass *adevc = ACPI_DEV_AML_IF_CLASS(klass);

    dc->desc = "Thunderbolt PCI Express Root Port";
    adevc->build_dev_aml = build_thunderbolt_root_port_aml;
}

static const TypeInfo thunderbolt_root_port_info = {
    .name       = TYPE_THUNDERBOLT_ROOT_PORT,
    .parent     = "pcie-root-port",
    .class_init = thunderbolt_root_port_class_init,
};

static void thunderbolt_root_port_register_types(void)
{
    type_register_static(&thunderbolt_root_port_info);
}

type_init(thunderbolt_root_port_register_types)
