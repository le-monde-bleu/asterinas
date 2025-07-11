// SPDX-License-Identifier: MPL-2.0

use alloc::{sync::Arc, vec::Vec};

use ostd::{bus::BusProbeError, sync::SpinLock};

use super::{
    bus::{
        bus::{MmioDevice, MmioDriver},
        common_device::MmioCommonDevice,
    },
    device::VirtioMmioTransport,
};

#[derive(Debug)]
pub struct VirtioMmioDriver {
    devices: SpinLock<Vec<VirtioMmioTransport>>,
}

impl VirtioMmioDriver {
    pub fn pop_device_transport(&self) -> Option<VirtioMmioTransport> {
        self.devices.lock().pop()
    }

    pub(super) fn new() -> Self {
        VirtioMmioDriver {
            devices: SpinLock::new(Vec::new()),
        }
    }
}

impl MmioDriver for VirtioMmioDriver {
    fn probe(
        &self,
        device: MmioCommonDevice,
    ) -> Result<Arc<dyn MmioDevice>, (BusProbeError, MmioCommonDevice)> {
        let device = VirtioMmioTransport::new(device);
        let mmio_device = device.mmio_device().clone();
        self.devices.lock().push(device);
        Ok(mmio_device)
    }
}
