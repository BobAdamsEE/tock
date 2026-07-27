//! Credential policy: sign what needs privilege, leave everything else alone.
//!
//! Tock splits two decisions that are easy to conflate:
//!
//! 1. **May this application run?** `AppCredentialsPolicy::require_credentials`.
//! 2. **What identity does it get?** `Compress::to_short_id`.
//!
//! Only the second is a privilege. This board answers "yes" to the first for
//! everything, so `blink` and any other unsigned application keep running, and
//! grants a *fixed* identity only to an application whose signature was
//! actually accepted. The reboot capsule keys off that identity.
//!
//! The safety of the arrangement rests on `ShortId`'s equality:
//!
//! ```text
//! (ShortId::Fixed(a), ShortId::Fixed(b)) => a == b,
//! _ => false,
//! ```
//!
//! An unsigned application is `LocallyUnique` and therefore matches nothing.
//! A mistake in the assigner denies privilege rather than granting it.

use kernel::process::{Process, ProcessBinary, ShortId};
use kernel::process_checker::{
    AppCredentialsPolicy, AppCredentialsPolicyClient, AppUniqueness, CheckResult, Compress,
};
use kernel::utilities::cells::OptionalCell;
use kernel::ErrorCode;
use tock_tbf::types::TbfFooterV2Credentials;

/// Wraps a credential checker so that a missing credential is not fatal.
///
/// Everything is delegated to the inner policy except `require_credentials`,
/// which reports `false`: an application with no credential, or whose
/// credentials were all passed over, still runs -- it simply ends up without
/// an accepted credential, and so without a fixed identity.
pub struct OptionalCredentials<'a> {
    inner: &'a dyn AppCredentialsPolicy<'a>,
    client: OptionalCell<&'a dyn AppCredentialsPolicyClient<'a>>,
}

impl<'a> OptionalCredentials<'a> {
    pub fn new(inner: &'a dyn AppCredentialsPolicy<'a>) -> OptionalCredentials<'a> {
        OptionalCredentials {
            inner,
            client: OptionalCell::empty(),
        }
    }

    /// Interpose on the inner policy's callbacks. Must be called once, with a
    /// `&'static` reference to this object.
    pub fn setup(&'a self) {
        self.inner.set_client(self);
    }
}

impl<'a> AppCredentialsPolicy<'a> for OptionalCredentials<'a> {
    fn set_client(&self, client: &'a dyn AppCredentialsPolicyClient<'a>) {
        self.client.set(client);
    }

    fn require_credentials(&self) -> bool {
        // The whole point: unsigned applications are still allowed to run.
        false
    }

    fn check_credentials(
        &self,
        credentials: TbfFooterV2Credentials,
        integrity_region: &'a [u8],
    ) -> Result<(), (ErrorCode, TbfFooterV2Credentials, &'a [u8])> {
        self.inner.check_credentials(credentials, integrity_region)
    }
}

impl<'a> AppCredentialsPolicyClient<'a> for OptionalCredentials<'a> {
    fn check_done(
        &self,
        result: Result<CheckResult, ErrorCode>,
        credentials: TbfFooterV2Credentials,
        integrity_region: &'a [u8],
    ) {
        self.client.map(move |client| {
            client.check_done(result, credentials, integrity_region);
        });
    }
}

/// Assigns a fixed identity only to applications with an accepted credential.
///
/// The identity is derived from the package name, which is safe to trust *here*
/// precisely because it is only consulted for a signed application: the
/// signature covers the integrity region, which includes the header the name
/// lives in. Deriving from the name rather than a single constant means several
/// signed applications can coexist -- the kernel refuses to start two processes
/// sharing a fixed `ShortId`.
pub struct SignedAppIdAssigner {}

impl SignedAppIdAssigner {
    pub fn new() -> SignedAppIdAssigner {
        SignedAppIdAssigner {}
    }

    /// FNV-1a over the package name. Any stable hash would do; this one is
    /// small and has no dependencies.
    pub fn name_id(name: &str) -> u32 {
        let mut hash: u32 = 0x811C_9DC5;
        for byte in name.as_bytes() {
            hash ^= *byte as u32;
            hash = hash.wrapping_mul(0x0100_0193);
        }
        // Zero is not a legal fixed ShortId.
        if hash == 0 {
            1
        } else {
            hash
        }
    }
}

/// Uniqueness is defined by the identity itself, so it cannot drift out of step
/// with [`Compress::to_short_id`]. Unsigned applications are `LocallyUnique`,
/// which compares unequal to everything, so any number of them coexist.
impl AppUniqueness for SignedAppIdAssigner {
    fn different_identifier(&self, a: &ProcessBinary, b: &ProcessBinary) -> bool {
        self.to_short_id(a) != self.to_short_id(b)
    }

    fn different_identifier_process(&self, a: &ProcessBinary, b: &dyn Process) -> bool {
        self.to_short_id(a) != b.short_app_id()
    }

    fn different_identifier_processes(&self, a: &dyn Process, b: &dyn Process) -> bool {
        a.short_app_id() != b.short_app_id()
    }
}

impl Compress for SignedAppIdAssigner {
    fn to_short_id(&self, process: &ProcessBinary) -> ShortId {
        // `get_credential` is `Some` only when a credential was *accepted*,
        // which for this board means a verified signature.
        match process.get_credential() {
            Some(_) => {
                let name = process.header.get_package_name().unwrap_or("");
                core::num::NonZeroU32::new(Self::name_id(name)).into()
            }
            None => ShortId::LocallyUnique,
        }
    }
}

// `AppIdPolicy` needs no impl: the kernel blanket-implements it for anything
// that is `AppUniqueness + Compress`.
