# How to Contribute Changes to the Robot URDF

Hi there, and thanks for your interest in improving our robot models!

The URDFs in this repo aren’t edited by hand, they’re automatically generated from YAML configuration files (see [creo2urdf/data/R1Mk3](https://github.com/mesh-iit/r1-models/tree/master/urdf/creo2urdf/data/R1Mk3)). So, if you’d like to adjust something and/or add a new feature in the robot’s description, the best way is to update these YAML files.

---

## 📝 Step-by-step guide to make changes to the URDF

1. **Fork this repository** and make your own feature branch.
2. **Find and edit the relevant `.yaml` file(s)** in your fork with your proposed changes.
3. **Push your changes** to your fork.
4. **Open a pull request** to this repository.

That’s it! When you open a PR, our GitHub Actions will automatically generate the updated URDF from your YAML changes and add it to your PR for you, so you don’t need to worry about building the URDF yourself.

If at any point you’re unsure about which file to change, or if you’re stuck, just [open an issue](https://github.com/mesh-iit/r1-models/issues). We’re happy to help and will get back to you as soon as possible.

Thanks again for contributing 🚀