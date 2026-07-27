# Third-party notices

`fv_kws` links the Vosk 0.3.45 Linux aarch64 library, uses
`vosk-model-small-ja-0.22`, and calls Vosk through the `vosk` Rust crate 0.3.1.

- Vosk source: https://github.com/alphacep/vosk-api
- Vosk models: https://alphacephei.com/vosk/models
- Rust wrapper: https://github.com/Bear-03/vosk-rs
- Vosk wheel SHA-256:
  `54efb47dd890e544e9e20f0316413acec7f8680d04ec095c6140ab4e70262704`
- model archive SHA-256:
  `efa092d280153a77615e9e0c7d7283e93e600de3d19d3bec686c57ef19d52eac`

The setup script downloads these upstream artifacts and rejects any digest
mismatch.
