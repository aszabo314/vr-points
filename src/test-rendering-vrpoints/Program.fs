open System
open Aardvark.Base
open Aardvark.Rendering
open Aardvark.SceneGraph
open Aardvark.Application
open Aardvark.Application.Slim
open FSharp.Data.Adaptive
open FSharp.Data.Adaptive.Operators
open Aardvark.Application.Utilities
open Aardvark.Rendering.PointSet
open Aardvark.Base.Ag

[<EntryPoint;STAThread>]
let main argv = 
    Aardvark.Init()

    let win = 
        window {
            display Display.Stereo
            
        }
               
    let store = @"C:\bla\stores\3-techzentrum_faro\"
    let key = Path.combine [System.IO.Path.GetDirectoryName store;"key.txt"] |> File.readAllText
    let inst = LodTreeInstance.load "source" key store [] |> Option.get
    let centerTrafo = Trafo3d.Translation -inst.root.WorldBoundingBox.Center
    let cfg = LodTreeRenderConfig.simple
    let sg = 
        Sg.lodTree cfg (ASet.single inst)
        |> Sg.shader {
            do! PointSetShaders.lodPointSize2
            do! PointSetShaders.lodPointCircular
        }
        |> Sg.trafo' centerTrafo
        |> Sg.uniform "PointSize" (AVal.constant 5.0)
        |> Sg.uniform "ViewportSize" win.Sizes
        |> Sg.uniform "PointVisualization" (PointVisualization.Color |> AVal.constant)
        |> Sg.uniform "MagicExp" (~~0.0)


    let combine (view : Trafo3d[]) (proj : Trafo3d[]) =
        let v =
            let c = view |> Array.map (fun v -> v.Backward.C3.XYZ) |> Array.average
            let f = view |> Array.map (fun v -> -v.Backward.C2.XYZ) |> Array.average
            let u = view |> Array.map (fun v -> v.Backward.C1.XYZ) |> Array.average
            CameraView.lookAt (c-f*0.1) (c+f) u |> CameraView.viewTrafo
        let p =
            Frustum.perspective 130.0 0.1 1000.0 1.0 |> Frustum.projTrafo
        v,p
            

    let vp = AVal.map2 combine win.View win.Proj
    sg?ViewTrafo <- AVal.map fst vp
    sg?ProjTrafo <- AVal.map snd vp

    win.Scene <- sg
    win.Run()
    0
